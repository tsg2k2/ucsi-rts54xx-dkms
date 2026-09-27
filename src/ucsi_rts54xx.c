// SPDX-License-Identifier: GPL-2.0-only
/*
 * UCSI I2C transport driver for Realtek RTS54xx USB-C PD controllers
 *
 * The RTS54xx is a PD controller (PDC) with a vendor SMBus command set rather
 * than a UCSI mailbox. Standard UCSI commands are tunnelled through vendor
 * command 0x0E; a few (PPM_RESET, ACK_CC_CI, SET_NOTIFICATION_ENABLE) need
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/unaligned.h>

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
	if (ret == -EREMOTEIO) {
		/*
		 * The ASUS firmware rejects GET_PDOS outright; report it as
		 * unsupported so the core does not chase it with
		 * GET_ERROR_STATUS.
		 */
		if (cmd == UCSI_GET_PDOS)
			return UCSI_CCI_COMMAND_COMPLETE |
			       UCSI_CCI_NOT_SUPPORTED;
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
	}
	if (ret < 0) {
		dev_dbg(&rts->client->dev, "UCSI command 0x%02x failed: %d\n",
			cmd, ret);
		return UCSI_CCI_COMMAND_COMPLETE | UCSI_CCI_ERROR;
	}

	if (cmd == UCSI_GET_CONNECTOR_STATUS && ret >= 2)
		rts->pending_change = get_unaligned_le16(rts->message_in);

	return UCSI_CCI_COMMAND_COMPLETE | UCSI_SET_CCI_LENGTH(ret);
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
	},
	.probe = ucsi_rts54_probe,
};
module_i2c_driver(ucsi_rts54_driver);

MODULE_DESCRIPTION("UCSI I2C transport driver for Realtek RTS54xx USB-C PD controllers");
MODULE_LICENSE("GPL");
