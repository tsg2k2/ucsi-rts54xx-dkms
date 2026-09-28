// SPDX-License-Identifier: GPL-2.0-only
/*
 * UCSI I2C transport driver for Realtek RTS54xx USB-C PD controllers
 *
 * The RTS54xx is a PD controller (PDC) with a vendor SMBus command set rather
 * than a UCSI mailbox. Standard UCSI commands are tunnelled through vendor
 * command 0x0E; a few (PPM_RESET, ACK_CC_CI, SET_NOTIFICATION_ENABLE, GET_PDOS) need
 * their own framing. Each command is written, its completion polled through a
 * one-byte ping status, and any response fetched with block-read command 0x80.
 * This driver runs that exchange synchronously and presents the result to the
 * UCSI core as CCI and MESSAGE_IN, emulating the PPM mailbox.
 *
 * Attention is signalled with an SMBus alert: the controller pulls its
 * interrupt line low until its address is read back from the Alert Response
 * Address.
 *
 * Protocol reference: ChromiumOS EC zephyr/drivers/usbc/pdc_rts54xx.c.
 */

#include <linux/acpi.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/unaligned.h>
#include <linux/usb.h>

#include "ucsi.h"

#define RTS54_ARA_ADDR		0x0c
#define RTS54_BLOCK_READ	0x80
#define RTS54_MAX_BLOCK		32

/* Ping status: cmd_sts in bits [1:0], data_len in bits [7:2] */
#define RTS54_PING_STS(p)	((p) & 0x3)
#define RTS54_PING_LEN(p)	((p) >> 2)
#define RTS54_STS_BUSY		0
#define RTS54_STS_DONE		1
#define RTS54_STS_DEFERRED	2
#define RTS54_STS_ERROR		3

#define RTS54_POLL_US		10000
#define RTS54_TIMEOUT_MS	1000

#define RTS54_CMD_UCSI		0x0e

#define RTS54_IC_STATUS_LEN	0x1f
#define RTS54_IC_FW_MAJOR	4
#define RTS54_IC_FW_MINOR	5
#define RTS54_IC_FW_PATCH	6
#define RTS54_IC_VID		10
#define RTS54_IC_PID		12
#define RTS54_REALTEK_VID	0x0bda

/* Vendor PD policy commands, command 0x08 */
#define RTS54_CMD_VENDOR		0x08
#define RTS54_SUB_TCPM_RESET		0x00
#define RTS54_SUB_SET_PDO		0x03
#define RTS54_SUB_SET_TPC_RP		0x05
#define RTS54_SUB_SET_TPC_RECONNECT	0x1f
#define RTS54_SUB_INIT_PD_AMS		0x20
#define RTS54_SUB_FORCE_POWER_SWITCH	0x21
#define RTS54_SUB_SET_TPC_DISCONNECT	0x23
#define RTS54_SUB_GET_PDO		0x83
#define RTS54_SUB_GET_RDO		0x84
#define RTS54_SUB_GET_TPC_RP		0x85
#define RTS54_SUB_GET_VDO		0x9a
#define RTS54_SUB_GET_PARTNER_SRC_PDO	0xa7

#define RTS54_MAX_PDOS		7
#define RTS54_PDO_SEL(src, partner, off, num) \
	((src) | (partner) << 1 | ((off) & 0x7) << 2 | ((num) & 0x7) << 5)

/* GET_RTK_STATUS (command 0x09): 14 bytes on firmware 2.13.x */
#define RTS54_CMD_RTK_STATUS	0x09
#define RTS54_RTK_STATUS_LEN	14
/* Byte offsets into the response, byte count stripped */
#define RTS54_ST_PORT		4	/* supply, op mode [3:1], PD cable [5] */
#define RTS54_ST_PD_CABLE	BIT(5)
#define RTS54_ST_OPMODE(b)	(((b) >> 1) & 0x7)
#define RTS54_OPMODE_PD		3

/* GET_VDO types (Realtek PD command interface) */
#define RTS54_VDO_ORIGIN_SOP	1
#define RTS54_VDO_ORIGIN_SOP_P	2
#define RTS54_ID_VDOS		6	/* ID header, cert, product, 3 x type */

/* A Discover Identity ACK header, for the emulated GET_PD_MESSAGE */
#define RTS54_DISC_ID_ACK_HDR	0xff008041

/* Alert storms without a matching ARA response before the IRQ is given up */
#define RTS54_IRQ_STORM_LIMIT	200

static const u8 rts54_vendor_cmd_enable[] = { 0x01, 0x03, 0xda, 0x0b, 0x01 };
static const u8 rts54_get_ic_status[] = { 0x3a, 0x03, 0x00, 0x00,
					  RTS54_IC_STATUS_LEN };
static const u8 rts54_ppm_reset[] = { RTS54_CMD_UCSI, 0x02, UCSI_PPM_RESET,
				      0x00 };

struct ucsi_rts54 {
	struct i2c_client *client;
	struct ucsi *ucsi;
	/* Serialises controller transactions and protects cci/message_in */
	struct mutex lock;
	u32 cci;
	u8 message_in[RTS54_MAX_BLOCK];
	/* Change bits from the last GET_CONNECTOR_STATUS, cleared on ACK */
	u16 pending_change;
	unsigned int irq_misses;
	bool irq_enabled;
	bool registered;
	/* Firmware source PDOs captured at probe, for source_pdos "restore" */
	u32 default_pdos[RTS54_MAX_PDOS];
	int num_default_pdos;
	bool disconnected;
	struct dentry *debugfs;
};

static int rts54_write(struct ucsi_rts54 *rts, const u8 *buf, int len)
{
	int ret;

	ret = i2c_master_send(rts->client, buf, len);
	if (ret < 0)
		return ret;
	return ret == len ? 0 : -EIO;
}

/*
 * Poll the ping status until the controller has finished the last command.
 * Returns the raw ping byte.
 */
static int rts54_wait(struct ucsi_rts54 *rts)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(RTS54_TIMEOUT_MS);
	u8 ping;
	int ret;

	do {
		usleep_range(RTS54_POLL_US, RTS54_POLL_US + 2000);

		ret = i2c_master_recv(rts->client, &ping, 1);
		if (ret < 0)
			return ret;
		if (ret != 1)
			return -EIO;

		switch (RTS54_PING_STS(ping)) {
		case RTS54_STS_DONE:
		case RTS54_STS_ERROR:
			return ping;
		}
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int rts54_block_read(struct ucsi_rts54 *rts, u8 *buf, int len)
{
	u8 cmd = RTS54_BLOCK_READ;
	struct i2c_msg msgs[] = {
		{
			.addr = rts->client->addr,
			.len = 1,
			.buf = &cmd,
		}, {
			.addr = rts->client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};
	int ret;

	ret = i2c_transfer(rts->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;
	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

/*
 * Run one controller command. On success returns the number of response
 * bytes copied to @resp; -EREMOTEIO if the controller reported CMD_ERROR.
 *
 * @hint_len is the response size to read when the ping status does not report
 * one: GET_IC_STATUS completes with data_len 0 on this firmware and carries its
 * length only in the block's byte-count prefix.
 */
static int rts54_exec(struct ucsi_rts54 *rts, const u8 *cmd, int cmd_len,
		      u8 *resp, int resp_len, int hint_len)
{
	u8 buf[RTS54_MAX_BLOCK];
	int ret, len;

	lockdep_assert_held(&rts->lock);

	ret = rts54_write(rts, cmd, cmd_len);
	if (ret)
		return ret;

	ret = rts54_wait(rts);
	if (ret < 0)
		return ret;
	if (RTS54_PING_STS(ret) == RTS54_STS_ERROR)
		return -EREMOTEIO;

	len = RTS54_PING_LEN(ret) ?: hint_len;
	if (!len || !resp)
		return 0;

	len = min(len, RTS54_MAX_BLOCK - 1);
	ret = rts54_block_read(rts, buf, len + 1);
	if (ret)
		return ret;

	/* First byte is the byte count of the response */
	len = min3((int)buf[0], len, resp_len);
	memcpy(resp, buf + 1, len);

	return len;
}

/* Bytes of UCSI command-specific data the controller expects per command */
static int rts54_ucsi_param_len(u8 cmd)
{
	switch (cmd) {
	case UCSI_CANCEL:
	case UCSI_GET_CAPABILITY:
		return 0;
	case UCSI_CONNECTOR_RESET:
	case UCSI_GET_CONNECTOR_CAPABILITY:
	case UCSI_GET_CAM_SUPPORTED:
	case UCSI_GET_CURRENT_CAM:
	case UCSI_GET_CABLE_PROPERTY:
	case UCSI_GET_CONNECTOR_STATUS:
	case UCSI_GET_ERROR_STATUS:
	case UCSI_GET_LPM_PPM_INFO:
		return 1;
	case UCSI_SET_CCOM:
	case UCSI_SET_UOR:
	case UCSI_SET_PDM:
	case UCSI_SET_PDR:
		return 2;
	case UCSI_GET_PDOS:
	case UCSI_READ_POWER_LEVEL:
		return 3;
	case UCSI_GET_ALTERNATE_MODES:
		return 4;
	default:
		return 6;
	}
}

static u32 rts54_ucsi_command(struct ucsi_rts54 *rts, u64 command)
{
	u8 cmd = UCSI_COMMAND(command);
	int plen = rts54_ucsi_param_len(cmd);
	u8 buf[4 + 6];
	int i, ret;

	buf[0] = RTS54_CMD_UCSI;
	buf[1] = plen + 2;
	buf[2] = cmd;
	buf[3] = 0;
	for (i = 0; i < plen; i++)
		buf[4 + i] = command >> (16 + 8 * i);

	ret = rts54_exec(rts, buf, 4 + plen, rts->message_in,
			 sizeof(rts->message_in), 0);
	if (ret == -EREMOTEIO)
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
	if (ret < 0) {
		dev_dbg(&rts->client->dev, "UCSI command 0x%02x failed: %d\n",
			cmd, ret);
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
	}

	if (cmd == UCSI_GET_CONNECTOR_STATUS && ret >= 2)
		rts->pending_change = get_unaligned_le16(rts->message_in);

	/* GET_PD_MESSAGE is emulated below; advertise it (features bit 8) */
	if (cmd == UCSI_GET_CAPABILITY && ret > 6)
		rts->message_in[6] |= UCSI_CAP_GET_PD_MESSAGE >> 8;

	return UCSI_CCI_COMMAND_COMPLETE | UCSI_SET_CCI_LENGTH(ret);
}

/*
 * The UCSI form of GET_PDOS (0x0E/0x10) is rejected by older firmware, so use
 * the vendor GET_PDO (0x08/0x83) that Realtek's own drivers use. Its selector
 * byte packs the same fields: bit 0 source, bit 1 partner, bits 4:2 offset,
 * bits 7:5 count. The response is the PDOs back to back, as in UCSI.
 */
static u32 rts54_get_pdos(struct ucsi_rts54 *rts, u64 command)
{
	bool partner = command & UCSI_GET_PDOS_PARTNER_PDO(1);
	bool source = command & UCSI_GET_PDOS_SRC_PDOS;
	unsigned int offset = (command >> 24) & 0xff;
	unsigned int num = ((command >> 32) & 0x3) + 1;
	u8 buf[5];
	int ret;

	/* The controller holds at most 7 PDOs per list */
	if (offset > 7)
		return UCSI_CCI_COMMAND_COMPLETE;
	num = min(num, 8 - offset);

	buf[0] = 0x08;
	buf[1] = 0x03;
	buf[2] = 0x83;
	buf[3] = 0x00;
	buf[4] = source | partner << 1 | (offset & 0x7) << 2 | (num & 0x7) << 5;

	ret = rts54_exec(rts, buf, sizeof(buf), rts->message_in,
			 num * sizeof(u32), 0);
	if (ret < 0) {
		dev_dbg(&rts->client->dev, "GET_PDO failed: %d\n", ret);
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
	}

	return UCSI_CCI_COMMAND_COMPLETE |
	       UCSI_SET_CCI_LENGTH(round_down(ret, sizeof(u32)));
}

static int rts54_rtk_status(struct ucsi_rts54 *rts, u8 *st)
{
	static const u8 cmd[] = { RTS54_CMD_RTK_STATUS, 0x03, 0x00, 0x00,
				  RTS54_RTK_STATUS_LEN };
	int ret;

	ret = rts54_exec(rts, cmd, sizeof(cmd), st, RTS54_RTK_STATUS_LEN, 0);
	if (ret < 0)
		return ret;
	return ret == RTS54_RTK_STATUS_LEN ? 0 : -EIO;
}

/* Read the six Discover Identity VDOs the controller cached for @origin */
static int rts54_get_id_vdos(struct ucsi_rts54 *rts, u8 origin, __le32 *vdo)
{
	u8 cmd[5 + RTS54_ID_VDOS] = {
		RTS54_CMD_VENDOR, 0x03 + RTS54_ID_VDOS, RTS54_SUB_GET_VDO, 0x00,
		RTS54_ID_VDOS | origin << 3,
	};
	int i, ret;

	for (i = 0; i < RTS54_ID_VDOS; i++)
		cmd[5 + i] = i + 1;	/* ID header, cert, product, type 1-3 */

	ret = rts54_exec(rts, cmd, sizeof(cmd), (u8 *)vdo,
			 RTS54_ID_VDOS * sizeof(u32), 0);
	if (ret < 0)
		return ret;
	return ret == RTS54_ID_VDOS * sizeof(u32) ? 0 : -EIO;
}

/*
 * UCSI 1.x has no GET_PD_MESSAGE, so partner and cable identity would stay
 * empty. Emulate the Discover Identity response from the vendor GET_VDO, as
 * ChromeOS does. The controller returns stale data when no identity was
 * discovered, so only trust SOP with a PD contract and SOP' with an e-marked
 * cable.
 */
static u32 rts54_get_pd_message(struct ucsi_rts54 *rts, u64 command)
{
	unsigned int recipient = (command >> 23) & 0x7;
	unsigned int offset = (command >> 26) & 0xff;
	unsigned int bytes = (command >> 34) & 0xff;
	unsigned int type = (command >> 42) & 0x3f;
	__le32 id[1 + RTS54_ID_VDOS] = {};
	u8 st[RTS54_RTK_STATUS_LEN];
	bool valid;
	int ret;

	if (type != UCSI_GET_PD_MESSAGE_TYPE_IDENTITY ||
	    (recipient != UCSI_RECIPIENT_SOP &&
	     recipient != UCSI_RECIPIENT_SOP_P))
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_NOT_SUPPORTED;

	if (offset >= sizeof(id))
		return UCSI_CCI_COMMAND_COMPLETE;
	bytes = min3(bytes, (unsigned int)sizeof(id) - offset, 16U);

	ret = rts54_rtk_status(rts, st);
	if (ret)
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;

	if (recipient == UCSI_RECIPIENT_SOP)
		valid = RTS54_ST_OPMODE(st[RTS54_ST_PORT]) == RTS54_OPMODE_PD;
	else
		valid = st[RTS54_ST_PORT] & RTS54_ST_PD_CABLE;

	if (valid) {
		ret = rts54_get_id_vdos(rts, recipient == UCSI_RECIPIENT_SOP ?
					RTS54_VDO_ORIGIN_SOP :
					RTS54_VDO_ORIGIN_SOP_P, &id[1]);
		if (ret)
			return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
		if (id[1])
			id[0] = cpu_to_le32(RTS54_DISC_ID_ACK_HDR);
		else
			memset(id, 0, sizeof(id));
	}

	memcpy(rts->message_in, (u8 *)id + offset, bytes);
	return UCSI_CCI_COMMAND_COMPLETE | UCSI_SET_CCI_LENGTH(bytes);
}

static u32 rts54_ack_cc_ci(struct ucsi_rts54 *rts, u64 command)
{
	bool cc = command & UCSI_ACK_COMMAND_COMPLETE;
	u8 buf[9];
	int ret;

	/* Command completion is purely host-side state in this emulation */
	if (!(command & UCSI_ACK_CONNECTOR_CHANGE))
		return UCSI_CCI_ACK_COMPLETE;

	buf[0] = 0x0a;
	buf[1] = 0x07;
	buf[2] = 0x00;
	buf[3] = 0x00;
	put_unaligned_le16(rts->pending_change, &buf[4]);
	buf[6] = 0x00;
	buf[7] = 0x00;
	buf[8] = cc;

	ret = rts54_exec(rts, buf, sizeof(buf), NULL, 0, 0);
	if (ret < 0)
		dev_dbg(&rts->client->dev, "ACK_CC_CI failed: %d\n", ret);

	rts->pending_change = 0;

	return UCSI_CCI_ACK_COMPLETE;
}

static u32 rts54_set_notification_enable(struct ucsi_rts54 *rts, u64 command)
{
	u32 bits = command >> 16;
	u8 buf[8];
	int ret;

	buf[0] = 0x08;
	buf[1] = 0x06;
	buf[2] = 0x01;
	buf[3] = 0x00;
	buf[4] = bits;
	buf[5] = bits >> 8;
	/* Bit 16 of the UCSI mask shares a byte with Realtek-specific bits */
	buf[6] = (bits >> 16) & 0x1;
	buf[7] = 0x00;

	ret = rts54_exec(rts, buf, sizeof(buf), NULL, 0, 0);
	if (ret < 0)
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;

	return UCSI_CCI_COMMAND_COMPLETE;
}

static u32 rts54_reset_ppm(struct ucsi_rts54 *rts)
{
	int ret;

	ret = rts54_exec(rts, rts54_ppm_reset, sizeof(rts54_ppm_reset),
			 NULL, 0, 0);
	if (ret < 0)
		dev_warn(&rts->client->dev, "PPM_RESET failed: %d\n", ret);

	/* Keep the vendor command set unlocked across the reset */
	ret = rts54_exec(rts, rts54_vendor_cmd_enable,
			 sizeof(rts54_vendor_cmd_enable), NULL, 0, 0);
	if (ret < 0)
		dev_warn(&rts->client->dev, "VENDOR_CMD_ENABLE failed: %d\n",
			 ret);

	rts->pending_change = 0;

	return UCSI_CCI_RESET_COMPLETE;
}

static int ucsi_rts54_read_version(struct ucsi *ucsi, u16 *version)
{
	/* GET_CONNECTOR_STATUS returns the 9-byte UCSI 1.x layout */
	*version = UCSI_VERSION_1_2;
	return 0;
}

static int ucsi_rts54_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct ucsi_rts54 *rts = ucsi_get_drvdata(ucsi);

	mutex_lock(&rts->lock);
	*cci = rts->cci;
	mutex_unlock(&rts->lock);

	return 0;
}

static int ucsi_rts54_read_message_in(struct ucsi *ucsi, void *val, size_t len)
{
	struct ucsi_rts54 *rts = ucsi_get_drvdata(ucsi);

	if (len > sizeof(rts->message_in))
		return -EINVAL;

	mutex_lock(&rts->lock);
	memcpy(val, rts->message_in, len);
	mutex_unlock(&rts->lock);

	return 0;
}

static int ucsi_rts54_async_control(struct ucsi *ucsi, u64 command)
{
	struct ucsi_rts54 *rts = ucsi_get_drvdata(ucsi);
	u32 cci;

	mutex_lock(&rts->lock);

	rts->cci = 0;
	memset(rts->message_in, 0, sizeof(rts->message_in));

	switch (UCSI_COMMAND(command)) {
	case UCSI_PPM_RESET:
		cci = rts54_reset_ppm(rts);
		break;
	case UCSI_ACK_CC_CI:
		cci = rts54_ack_cc_ci(rts, command);
		break;
	case UCSI_SET_NOTIFICATION_ENABLE:
		cci = rts54_set_notification_enable(rts, command);
		break;
	case UCSI_GET_PDOS:
		cci = rts54_get_pdos(rts, command);
		break;
	case UCSI_GET_PD_MESSAGE:
		cci = rts54_get_pd_message(rts, command);
		break;
	default:
		cci = rts54_ucsi_command(rts, command);
		break;
	}

	rts->cci = cci;
	mutex_unlock(&rts->lock);

	ucsi_notify_common(ucsi, cci);

	return 0;
}

static const struct ucsi_operations ucsi_rts54_ops = {
	.read_version = ucsi_rts54_read_version,
	.read_cci = ucsi_rts54_read_cci,
	.poll_cci = ucsi_rts54_read_cci,
	.read_message_in = ucsi_rts54_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = ucsi_rts54_async_control,
};

static int rts54_read_ara(struct ucsi_rts54 *rts, u8 *ara)
{
	struct i2c_msg msg = {
		.addr = RTS54_ARA_ADDR,
		.flags = I2C_M_RD,
		.len = 1,
		.buf = ara,
	};
	int ret;

	ret = i2c_transfer(rts->client->adapter, &msg, 1);
	if (ret < 0)
		return ret;
	return ret == 1 ? 0 : -EIO;
}

static irqreturn_t ucsi_rts54_irq(int irq, void *data)
{
	struct ucsi_rts54 *rts = data;
	bool ours = false;
	u8 ara;
	int i;

	/*
	 * Every alerting device on the bus answers the ARA in turn; keep
	 * reading until nobody does so the line is released.
	 */
	for (i = 0; i < 8; i++) {
		if (rts54_read_ara(rts, &ara))
			break;
		if ((ara >> 1) == rts->client->addr)
			ours = true;
	}

	if (!ours) {
		if (++rts->irq_misses == RTS54_IRQ_STORM_LIMIT) {
			dev_warn(&rts->client->dev,
				 "interrupt not acknowledged via ARA, disabling it; connector changes will not be reported\n");
			rts->irq_enabled = false;
			disable_irq_nosync(irq);
		}
		return IRQ_NONE;
	}

	rts->irq_misses = 0;
	/* Single-connector controller: report a change on connector 1 */
	ucsi_notify_common(rts->ucsi, 1 << 1);

	return IRQ_HANDLED;
}

static int rts54_init_chip(struct ucsi_rts54 *rts)
{
	struct device *dev = &rts->client->dev;
	u8 st[RTS54_IC_STATUS_LEN];
	u16 vid, pid;
	int ret;

	mutex_lock(&rts->lock);

	ret = rts54_exec(rts, rts54_vendor_cmd_enable,
			 sizeof(rts54_vendor_cmd_enable), NULL, 0, 0);
	if (ret < 0) {
		dev_err(dev, "VENDOR_CMD_ENABLE failed: %d\n", ret);
		goto out;
	}

	ret = rts54_exec(rts, rts54_get_ic_status, sizeof(rts54_get_ic_status),
			 st, sizeof(st), RTS54_IC_STATUS_LEN);
	if (ret < 0) {
		dev_err(dev, "GET_IC_STATUS failed: %d\n", ret);
		goto out;
	}
	if (ret < RTS54_IC_PID + 1) {
		dev_err(dev, "short GET_IC_STATUS response (%d bytes)\n", ret);
		ret = -EIO;
		goto out;
	}

	vid = get_unaligned_le16(&st[RTS54_IC_VID - 1]);
	pid = get_unaligned_le16(&st[RTS54_IC_PID - 1]);
	if (vid != RTS54_REALTEK_VID) {
		dev_err(dev, "unexpected vendor id 0x%04x\n", vid);
		ret = -ENODEV;
		goto out;
	}

	dev_info(dev, "Realtek PD controller %04x:%04x, firmware %u.%u.%u\n",
		 vid, pid, st[RTS54_IC_FW_MAJOR - 1], st[RTS54_IC_FW_MINOR - 1],
		 st[RTS54_IC_FW_PATCH - 1]);
	ret = 0;
out:
	mutex_unlock(&rts->lock);
	return ret;
}

/*
 * Port controls. These drive the controller's PD policy directly, outside the
 * UCSI core; the controller reports the resulting connector changes through
 * the usual alert.
 */

static int rts54_cmd(struct ucsi_rts54 *rts, const u8 *cmd, int len,
		     u8 *resp, int resp_len)
{
	int ret;

	mutex_lock(&rts->lock);
	ret = rts54_exec(rts, cmd, len, resp, resp_len, 0);
	mutex_unlock(&rts->lock);

	return ret;
}

static int rts54_vendor(struct ucsi_rts54 *rts, u8 sub, int nargs, u8 arg)
{
	u8 cmd[] = { RTS54_CMD_VENDOR, 0x02 + nargs, sub, 0x00, arg };

	return rts54_cmd(rts, cmd, 4 + nargs, NULL, 0);
}

static int rts54_read_pdos(struct ucsi_rts54 *rts, bool partner, u32 *pdos)
{
	u8 cmd[] = { RTS54_CMD_VENDOR, 0x03, RTS54_SUB_GET_PDO, 0x00,
		     RTS54_PDO_SEL(1, partner, 0, RTS54_MAX_PDOS) };
	__le32 raw[RTS54_MAX_PDOS];
	int i, ret;

	ret = rts54_cmd(rts, cmd, sizeof(cmd), (u8 *)raw, sizeof(raw));
	if (ret < 0)
		return ret;

	for (i = 0; i < ret / 4; i++)
		pdos[i] = le32_to_cpu(raw[i]);
	return ret / 4;
}

static int rts54_write_pdos(struct ucsi_rts54 *rts, const u32 *pdos, int n)
{
	u8 cmd[5 + 4 * RTS54_MAX_PDOS] = {
		RTS54_CMD_VENDOR, 0x03 + 4 * n, RTS54_SUB_SET_PDO, 0x00,
		(n & 0x7) | BIT(3),	/* count, source PDOs */
	};
	int i, ret;

	for (i = 0; i < n; i++)
		put_unaligned_le32(pdos[i], &cmd[5 + 4 * i]);

	ret = rts54_cmd(rts, cmd, 5 + 4 * n, NULL, 0);
	if (ret < 0)
		return ret;

	/* Re-advertise so an existing contract picks up the new list */
	rts54_vendor(rts, RTS54_SUB_INIT_PD_AMS, 1, 0x04);
	return 0;
}

static int rts54_format_pdo(char *buf, int at, u32 pdo)
{
	switch (pdo >> 30) {
	case 0:
		return sysfs_emit_at(buf, at, "0x%08x fixed %umV %umA\n", pdo,
				     ((pdo >> 10) & 0x3ff) * 50,
				     (pdo & 0x3ff) * 10);
	case 3:
		if (!((pdo >> 28) & 0x3))
			return sysfs_emit_at(buf, at,
					     "0x%08x pps %umV-%umV %umA\n", pdo,
					     ((pdo >> 8) & 0xff) * 100,
					     ((pdo >> 17) & 0xff) * 100,
					     (pdo & 0x7f) * 50);
		fallthrough;
	default:
		return sysfs_emit_at(buf, at, "0x%08x\n", pdo);
	}
}

static ssize_t reconnect_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	bool val;
	int ret;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_RECONNECT, 1, 0x01);
	if (ret < 0)
		return ret;
	rts->disconnected = false;
	return count;
}
static DEVICE_ATTR_WO(reconnect);

static ssize_t disconnect_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", rts->disconnected);
}

static ssize_t disconnect_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	bool val;
	int ret;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val)
		ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_DISCONNECT, 0, 0);
	else
		ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_RECONNECT, 1, 0x01);
	if (ret < 0)
		return ret;
	rts->disconnected = val;
	return count;
}
static DEVICE_ATTR_RW(disconnect);

/*
 * Detach the port at the Type-C level, which makes the controller drop VBUS,
 * then attach again: a software unplug/replug of whatever is connected.
 */
static ssize_t power_cycle_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	unsigned int ms;
	int ret;

	if (kstrtouint(buf, 0, &ms))
		return -EINVAL;
	if (ms == 1)
		ms = 1000;
	if (ms < 100 || ms > 10000)
		return -ERANGE;

	ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_DISCONNECT, 0, 0);
	if (ret < 0)
		return ret;
	rts->disconnected = true;

	msleep(ms);

	ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_RECONNECT, 1, 0x01);
	if (ret < 0)
		return ret;
	rts->disconnected = false;
	return count;
}
static DEVICE_ATTR_WO(power_cycle);

static const char * const rts54_rp_names[] = {
	[1] = "default", [2] = "1.5A", [3] = "3.0A",
};

static ssize_t tpc_rp_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	u8 cmd[] = { RTS54_CMD_VENDOR, 0x02, RTS54_SUB_GET_TPC_RP, 0x00 };
	u8 rp;
	int ret, tpc, pd;

	ret = rts54_cmd(rts, cmd, sizeof(cmd), &rp, 1);
	if (ret < 0)
		return ret;
	if (ret < 1)
		return -EIO;

	tpc = (rp >> 2) & 0x3;
	pd = (rp >> 4) & 0x3;
	return sysfs_emit(buf, "typec=%s pd=%s\n",
			  rts54_rp_names[tpc] ?: "reserved",
			  rts54_rp_names[pd] ?: "reserved");
}

static ssize_t tpc_rp_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	int i, ret;

	for (i = 1; i < ARRAY_SIZE(rts54_rp_names); i++)
		if (sysfs_streq(buf, rts54_rp_names[i]))
			break;
	if (i == ARRAY_SIZE(rts54_rp_names))
		return -EINVAL;

	ret = rts54_vendor(rts, RTS54_SUB_SET_TPC_RP, 1, i << 2 | i << 4);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(tpc_rp);

static ssize_t rdo_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	u8 cmd[] = { RTS54_CMD_VENDOR, 0x02, RTS54_SUB_GET_RDO, 0x00 };
	__le32 raw;
	u32 rdo;
	int ret;

	ret = rts54_cmd(rts, cmd, sizeof(cmd), (u8 *)&raw, sizeof(raw));
	if (ret == -EREMOTEIO || (ret >= 0 && ret < 4) || (ret >= 4 && !raw))
		return sysfs_emit(buf, "none\n");
	if (ret < 0)
		return ret;

	rdo = le32_to_cpu(raw);
	return sysfs_emit(buf, "0x%08x pdo=%u operating=%umA max=%umA\n", rdo,
			  (rdo >> 28) & 0x7, ((rdo >> 10) & 0x3ff) * 10,
			  (rdo & 0x3ff) * 10);
}
static DEVICE_ATTR_RO(rdo);

static ssize_t source_pdos_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	u32 pdos[RTS54_MAX_PDOS];
	int i, n, len = 0;

	n = rts54_read_pdos(rts, false, pdos);
	if (n < 0)
		return n;

	for (i = 0; i < n; i++)
		len += rts54_format_pdo(buf, len, pdos[i]);
	return len;
}

/*
 * Write "restore" or up to seven PDOs in hex. PD requires the first to be the
 * fixed vSafe5V supply.
 */
static ssize_t source_pdos_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	u32 pdos[RTS54_MAX_PDOS];
	char *copy, *p, *tok;
	int n = 0, ret;

	if (sysfs_streq(buf, "restore")) {
		if (!rts->num_default_pdos)
			return -ENODATA;
		ret = rts54_write_pdos(rts, rts->default_pdos,
				       rts->num_default_pdos);
		return ret < 0 ? ret : count;
	}

	copy = kstrndup(buf, count, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	p = copy;
	while ((tok = strsep(&p, " ,\t\n"))) {
		if (!*tok)
			continue;
		if (n == RTS54_MAX_PDOS || kstrtou32(tok, 16, &pdos[n])) {
			kfree(copy);
			return -EINVAL;
		}
		n++;
	}
	kfree(copy);

	if (!n || pdos[0] >> 30 || ((pdos[0] >> 10) & 0x3ff) != 100)
		return -EINVAL;

	ret = rts54_write_pdos(rts, pdos, n);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(source_pdos);

static ssize_t partner_source_pdo_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	u8 cmd[] = { RTS54_CMD_VENDOR, 0x02, RTS54_SUB_GET_PARTNER_SRC_PDO,
		     0x00 };
	__le32 raw;
	int ret;

	ret = rts54_cmd(rts, cmd, sizeof(cmd), (u8 *)&raw, sizeof(raw));
	if (ret == -EREMOTEIO || (ret >= 0 && ret < 4) || (ret >= 4 && !raw))
		return sysfs_emit(buf, "none\n");
	if (ret < 0)
		return ret;

	return rts54_format_pdo(buf, 0, le32_to_cpu(raw));
}
static DEVICE_ATTR_RO(partner_source_pdo);

static const struct {
	const char *name;
	u8 code;
} rts54_pd_ams[] = {
	{ "source_cap", 0x04 },
	{ "soft_reset", 0x06 },
	{ "hard_reset", 0x07 },
	{ "goto_min", 0x08 },
	{ "get_sink_cap", 0x09 },
	{ "get_source_cap", 0x0a },
};

static ssize_t pd_ams_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rts54_pd_ams); i++)
		if (sysfs_streq(buf, rts54_pd_ams[i].name))
			break;
	if (i == ARRAY_SIZE(rts54_pd_ams))
		return -EINVAL;

	ret = rts54_vendor(rts, RTS54_SUB_INIT_PD_AMS, 1, rts54_pd_ams[i].code);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_WO(pd_ams);

static ssize_t tcpm_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	bool val;
	int ret;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	ret = rts54_vendor(rts, RTS54_SUB_TCPM_RESET, 1, 0x01);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_WO(tcpm_reset);

static struct attribute *rts54_attrs[] = {
	&dev_attr_reconnect.attr,
	&dev_attr_disconnect.attr,
	&dev_attr_power_cycle.attr,
	&dev_attr_tpc_rp.attr,
	&dev_attr_rdo.attr,
	&dev_attr_source_pdos.attr,
	&dev_attr_partner_source_pdo.attr,
	&dev_attr_pd_ams.attr,
	&dev_attr_tcpm_reset.attr,
	NULL
};

static const struct attribute_group rts54_group = {
	.name = "rts54xx",
	.attrs = rts54_attrs,
};

static const struct attribute_group *rts54_groups[] = {
	&rts54_group,
	NULL
};

static int rts54_rtk_status_show(struct seq_file *s, void *unused)
{
	struct ucsi_rts54 *rts = s->private;
	u8 st[RTS54_RTK_STATUS_LEN];
	int ret;

	mutex_lock(&rts->lock);
	ret = rts54_rtk_status(rts, st);
	mutex_unlock(&rts->lock);
	if (ret)
		return ret;

	seq_printf(s, "%*ph\n", RTS54_RTK_STATUS_LEN, st);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rts54_rtk_status);

/*
 * Raw FORCE_SET_POWER_SWITCH byte (bit 6 + [1:0] VBSIN, bit 7 + [3:2] LP).
 * On boards where the source path is an external switch this has no effect;
 * use power_cycle instead.
 */
static ssize_t rts54_power_switch_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct ucsi_rts54 *rts = file->private_data;
	u8 val;
	int ret;

	ret = kstrtou8_from_user(ubuf, count, 0, &val);
	if (ret)
		return ret;

	ret = rts54_vendor(rts, RTS54_SUB_FORCE_POWER_SWITCH, 1, val);
	return ret < 0 ? ret : count;
}

static const struct file_operations rts54_power_switch_fops = {
	.open = simple_open,
	.write = rts54_power_switch_write,
	.llseek = noop_llseek,
};

static void rts54_debugfs_remove(void *data)
{
	struct ucsi_rts54 *rts = data;

	debugfs_remove_recursive(rts->debugfs);
}

static void rts54_debugfs_init(struct ucsi_rts54 *rts)
{
	struct device *dev = &rts->client->dev;
	char name[64];

	snprintf(name, sizeof(name), "rts54xx-%s", dev_name(dev));
	rts->debugfs = debugfs_create_dir(name, usb_debug_root);
	debugfs_create_file("rtk_status", 0400, rts->debugfs, rts,
			    &rts54_rtk_status_fops);
	debugfs_create_file("force_power_switch", 0200, rts->debugfs, rts,
			    &rts54_power_switch_fops);
	devm_add_action_or_reset(dev, rts54_debugfs_remove, rts);
}

static void ucsi_rts54_destroy(void *data)
{
	struct ucsi_rts54 *rts = data;

	if (rts->registered)
		ucsi_unregister(rts->ucsi);
	ucsi_destroy(rts->ucsi);
}

static int ucsi_rts54_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ucsi_rts54 *rts;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter does not support I2C transfers\n");

	rts = devm_kzalloc(dev, sizeof(*rts), GFP_KERNEL);
	if (!rts)
		return -ENOMEM;

	rts->client = client;
	mutex_init(&rts->lock);
	i2c_set_clientdata(client, rts);

	ret = rts54_init_chip(rts);
	if (ret)
		return dev_err_probe(dev, ret, "controller init failed\n");

	ret = rts54_read_pdos(rts, false, rts->default_pdos);
	if (ret > 0)
		rts->num_default_pdos = ret;
	else
		dev_warn(dev, "could not read source PDOs: %d\n", ret);

	rts54_debugfs_init(rts);

	rts->ucsi = ucsi_create(dev, &ucsi_rts54_ops);
	if (IS_ERR(rts->ucsi))
		return dev_err_probe(dev, PTR_ERR(rts->ucsi),
				     "failed to create UCSI interface\n");

	ret = devm_add_action_or_reset(dev, ucsi_rts54_destroy, rts);
	if (ret)
		return ret;

	ucsi_set_drvdata(rts->ucsi, rts);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						ucsi_rts54_irq, IRQF_ONESHOT,
						dev_name(dev), rts);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request IRQ\n");
		rts->irq_enabled = true;
	} else {
		dev_warn(dev, "no IRQ; connector changes will not be reported\n");
	}

	ret = ucsi_register(rts->ucsi);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register UCSI interface\n");

	rts->registered = true;
	return 0;
}

static int ucsi_rts54_suspend(struct device *dev)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	int ret;

	if (rts->irq_enabled)
		disable_irq(rts->client->irq);
	ret = ucsi_suspend(rts->ucsi);
	if (ret && rts->irq_enabled)
		enable_irq(rts->client->irq);

	return ret;
}

static int ucsi_rts54_resume(struct device *dev)
{
	struct ucsi_rts54 *rts = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&rts->lock);
	ret = rts54_exec(rts, rts54_vendor_cmd_enable,
			 sizeof(rts54_vendor_cmd_enable), NULL, 0, 0);
	mutex_unlock(&rts->lock);
	if (ret < 0)
		dev_warn(dev, "VENDOR_CMD_ENABLE on resume failed: %d\n", ret);

	if (rts->irq_enabled)
		enable_irq(rts->client->irq);
	return ucsi_resume(rts->ucsi);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ucsi_rts54_pm, ucsi_rts54_suspend,
				ucsi_rts54_resume);

static const struct acpi_device_id ucsi_rts54_acpi_ids[] = {
	{ "RTK5452" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ucsi_rts54_acpi_ids);

static struct i2c_driver ucsi_rts54_driver = {
	.driver = {
		.name = "ucsi_rts54xx",
		.acpi_match_table = ucsi_rts54_acpi_ids,
		.pm = pm_sleep_ptr(&ucsi_rts54_pm),
		.dev_groups = rts54_groups,
	},
	.probe = ucsi_rts54_probe,
};
module_i2c_driver(ucsi_rts54_driver);

MODULE_DESCRIPTION("UCSI I2C transport driver for Realtek RTS54xx USB-C PD controllers");
MODULE_LICENSE("GPL");
