# ucsi-rts54xx-dkms

A Linux UCSI transport driver, packaged for DKMS, for the **Realtek RTS54xx** USB-C/PD controllers that ASUS boards expose in ACPI as **`RTK5452`**.

Without this driver the controller has no Linux driver, and its port is missing from `/sys/class/typec/`. With it the port shows up in `typec`, with its data and power role, power mode, partner, and cable, and its PD source capabilities appear under `/sys/class/usb_power_delivery/`.

Power delivery works either way, because the controller negotiates in its own firmware. The driver only makes that state visible to Linux.

No upstream driver exists for this chip, and none has been proposed. This one is new: its protocol is taken from the open-source ChromiumOS EC driver `zephyr/drivers/usbc/pdc_rts54xx.c`, and it runs on top of the kernel's UCSI core.

## Hardware

Tested on **ASUS ProArt X870E-Creator WIFI** (BIOS 2401):

| | |
|---|---|
| ACPI | `\_SB.I2CA.TYPC`, `_HID "RTK5452"` |
| I2C | `0x65` command address, `0x0C` SMBus Alert Response Address, 400 kHz |
| IRQ | GPIO pin `0x54`, level, active-low |
| Chip | VID `0x0BDA`, PID `0x5453`, firmware 2.13.16 |
| Connectors | **1**: the front-panel USB 20Gbps Type-C header **U20G_C6**, PD 3.0, up to 30 W (5/9 V 3 A, 12 V 2.5 A, 15 V 2 A) |

The rear 20 Gbps Type-C port (C18) is *not* on this controller; the manual lists it as 5 V/3 A only. The two USB4 ports are on an ITE8853; see [ucsi-ite-dkms](https://github.com/tsg2k2/ucsi-ite-dkms).

Other ASUS boards that describe an `RTK5452` device will probably work too, but only this one has been tested.

## How it works

The RTS54xx does not implement a UCSI mailbox. It is a PD controller with a vendor SMBus command set:

1. **Unlock.** `VENDOR_CMD_ENABLE` (`01 03 DA 0B 01`) must be sent before the chip accepts anything else; until then every command returns `CMD_ERROR`. The driver sends it at probe, after every `PPM_RESET`, and on resume.
2. **Command.** Standard UCSI commands are wrapped as `0E <len> <ucsi-cmd> 00 <params…>`. Four commands need their own vendor framing: `ACK_CC_CI` (`0A 07 …`), `SET_NOTIFICATION_ENABLE` (`08 06 01 …`), `GET_PDOS` (`08 03 83 00 <sel>`), and `PPM_RESET`.
3. **Completion.** The driver polls a 1-byte ping status: `cmd_sts` in bits [1:0] (BUSY / DONE / DEFERRED / ERROR) and `data_len` in bits [7:2].
4. **Response.** It then block-reads with command `0x80`; the first byte of the response is its byte count.
5. **Events.** When something changes, the chip pulls the GPIO interrupt low. The driver reads the Alert Response Address (`0x0C`), which releases the line, and reports a connector change to the UCSI core.

The driver runs each command synchronously inside `async_control` and stores the result as CCI and MESSAGE_IN, so the UCSI core sees an ordinary PPM. It reports **UCSI 1.2**, because `GET_CONNECTOR_STATUS` returns the 9-byte 1.x layout.

### Partner and cable identity

UCSI 1.x has no `GET_PD_MESSAGE`, so the core can't read Discover Identity and `portN-partner/identity` would stay empty. The driver advertises the capability and answers the command itself from the vendor `GET_VDO` (`08 <3+n> 9A 00 <n|origin<<3> <types…>`), the same way ChromeOS does. The controller returns stale VDOs when nothing was discovered, so SOP identity is only reported while there is a PD contract, and SOP′ only with an e-marked cable. Otherwise the identity is all zeros.

## Port controls

The driver adds controls under `/sys/bus/i2c/devices/i2c-RTK5452:00/rts54xx/`. They talk to the controller's PD policy directly, and the resulting attach/detach events reach `typec` through the normal alert path.

| File | | What it does |
|---|---|---|
| `power_cycle` | W | Software unplug/replug: `SET_TPC_DISCONNECT`, hold, `SET_TPC_RECONNECT`. `echo 1` holds for `power_cycle_ms`; `echo <ms>` (100–30000) holds that long. VBUS is off for the hold, so the device's capacitors discharge and its controller resets. Measured on the ProArt C6 port: USB disconnect to next enumeration = hold + ~0.68 s |
| `power_cycle_ms` | RW | Default hold for `echo 1 > power_cycle`, in ms (default 2000, 100–30000) |
| `reconnect` | W | `echo 1`: `SET_TPC_RECONNECT` (`08 03 1F 00 01`). On a connected port the controller detaches and re-attaches by itself with a fixed ~0.64 s off time (the argument byte makes no difference; 0x01, 0x05, 0x0a and 0x32 all measured the same). After `disconnect` it re-attaches the port |
| `disconnect` | W | `echo 1` detaches the port and keeps VBUS off (`SET_TPC_DISCONNECT`, `08 02 23 00`, from Realtek's BSP command table, where it is defined but never called); `echo 0` or `reconnect` re-attaches |
| `tpc_rp` | RW | Advertised Type-C / PD Rp current: `default`, `1.5A`, `3.0A` (`GET/SET_TPC_RP`, `08 02 85` / `08 03 05`) |
| `source_pdos` | RW | Read: the source PDOs, decoded. Write: up to 7 hex PDOs (the first must be fixed 5 V), or `restore` for the firmware list captured at probe. Uses `SET_PDO` (`08 <3+4n> 03 00 <n\|src<<3> …`), then re-sends Source_Capabilities |
| `rdo` | R | The partner's current request (`GET_RDO`, `08 02 84 00`), or `none` without a PD contract |
| `partner_source_pdo` | R | `GET_CURRENT_PARTNER_SRC_PDO` (`08 02 A7 00`); only meaningful on a sink-capable port |
| `pd_ams` | W | Start a PD sequence: `source_cap`, `soft_reset`, `hard_reset`, `goto_min`, `get_sink_cap`, `get_source_cap` (`INIT_PD_AMS`, `08 03 20 00 <n>`) |
| `tcpm_reset` | W | `echo 1`: reset the controller's PD stack (`08 03 00 00 01`) |

`debugfs` (`/sys/kernel/debug/usb/rts54xx-i2c-RTK5452:00/`):

- `rtk_status`: raw 14-byte `GET_RTK_STATUS` block.
- `force_power_switch`: raw `FORCE_SET_POWER_SWITCH` byte (bit 6 + [1:0] = VBSIN, bit 7 + [3:2] = LP). On the ProArt both switch states read 0 while the port sources 3 A, which suggests VBUS goes through an external switch that this command doesn't control. Use `power_cycle` instead. This one is experimental.

The command framings come from Realtek's own 6.6 BSP driver (`drivers/usb/typec/rts54xx.c`) and ChromiumOS `pdc_rts54xx.c`.

### Firmware quirks

- **The UCSI form of `GET_PDOS` (`0E 05 10 …`) is rejected** with `CMD_ERROR`. The driver uses the vendor `GET_PDO` (`08 03 83 00 <sel>`) instead, which Realtek's own drivers use. Its selector byte is `source | partner<<1 | offset<<2 | count<<5`, and the response is the PDOs back to back, as in UCSI.
- **`GET_IC_STATUS`** accepts a length of at most 31 (`0x1F`), completes with `data_len = 0`, and ignores the offset byte.
- The connector-number field is ignored, because the controller has only one connector.
- `GET_RTK_STATUS` ends at byte 14, so the power-reading fields (average current, voltage) that newer firmware has at bytes 15–19 are missing. `READ_POWER_LEVEL` and `GET_POWER_SWITCH_STATE` are rejected. **There is no way to read a live wattage** on this firmware; `hwmon ucsi_source_psy_*` shows only the negotiated contract.
- `GET_TPC_RP` returns `0x3f`: Type-C Rp and PD Rp are both 3 (3.0 A).

## Status

| | |
|---|---|
| Probe, unlock, chip identification | ✅ |
| PPM reset, capabilities, connector capability/status, cable | ✅ `/sys/class/typec/portN` with partner and cable |
| Unload/reload | ✅ attached device not disturbed |
| Hot-plug events (IRQ + ARA) | ✅ on unplug/replug the partner and cable are removed and re-created about 1 s after USB enumeration; about 5 interrupts per replug, no storm |
| Suspend/resume | ⚠️ untested |
| Read-only controls (`tpc_rp`, `rdo`, `source_pdos` read, `partner_source_pdo`, `rtk_status`) | ✅ framing verified on the chip; the driver paths are untested until the next reload |
| Identity emulation (`GET_PD_MESSAGE` via `GET_VDO`) | ⚠️ `GET_VDO` verified on the chip (port identity VID 0x0BDA / PID 0x5450); a PD partner hasn't been tested yet |
| `power_cycle`, `reconnect`, `disconnect` | ✅ verified 2026-10-05 on C6 with an RTL8159 10G NIC: it drops and comes back at Gen 2x2 every time; off time = hold + ~0.68 s (3 s → 3.68 s, 8 s → 8.68 s) |
| `tcpm_reset` | ✅ verified 2026-10-05: the port detaches and re-attaches in ~1.4 s. The controller leaves the ping status at CMD_DEFERRED for this command (the reset takes the command state with it), so the driver takes DEFERRED as accepted; before 0.6 it reported a false I/O error / timeout. ⚠️ After a `reconnect` or `power_cycle`, the next `tcpm_reset` makes the controller send USB-PD Hard Resets to a non-PD device, which never answers: 3 VBUS cycles 6.0 s apart (tNoResponse + nHardResetCount = 2), reported as PD_RESET_COMPLETE. A `tcpm_reset` without a preceding reconnect is a single cycle |
| `tpc_rp` write | ✅ verified 2026-10-05: 3.0A ↔ 1.5A. The controller only flags a connector change when Rp goes down, so the driver reports a power op mode change itself and the typec `power_operation_mode` follows both ways (stale after an increase before 0.7) |
| `source_pdos` write, `pd_ams` | ⚠️ written, not yet exercised: they need a PD-capable sink on the port |
| Source/sink/partner PDOs | ✅ via vendor `GET_PDO`; on the ProArt, C6 advertises 5 V 3 A, 9 V 3 A, 12 V 2.5 A, 15 V 2 A, PPS 5–11 V 3 A, PPS 5–16 V 2 A |

If the interrupt line keeps firing without the chip answering the ARA, the driver disables the IRQ after 200 misses in a row and logs a warning. The port stays registered, but it won't report changes.

## Supported kernels

Built against the UCSI core's private `ucsi.h`, with an exact copy bundled for each series. This works the same way as ucsi-ite-dkms:

| Series | Header source | Tested |
|---|---|---|
| 7.2 | `v7.2.6` | ✅ 7.2.6 |
| 7.3 | `v7.3-rc4` | build only; re-check against 7.3 final |

The build refuses a kernel if there is no header for its series, or if the kernel does not export `ucsi_write_message_out_command`. That second check catches distro kernels whose UCSI core matches no upstream header, such as Ubuntu 7.0. `BUILD_EXCLUSIVE_KERNEL` makes DKMS skip those kernels.

**Adding a series:** copy `drivers/usb/typec/ucsi/ucsi.h` from that exact kernel tag into `src/headers/<series>/`, then extend `BUILD_EXCLUSIVE_KERNEL` in `src/dkms.conf`.

## Install

Requires `dkms` and the headers for your kernel.

```sh
./install.sh      # copies src/ to /usr/src/ucsi-rts54xx-<ver>, dkms install, modprobe
./uninstall.sh    # removes all versions
```

On a Secure Boot system, DKMS signs the module with its MOK key (`/var/lib/shim-signed/mok/`), and that key must be enrolled.

To build by hand without DKMS: `make -C src` (use `KVER=` to target another installed kernel).

## Debugging

The UCSI core's debugfs interface can send raw commands:

```sh
D=/sys/kernel/debug/usb/ucsi/i2c-RTK5452:00
echo 0x10012 | sudo tee $D/command && sudo cat $D/response   # GET_CONNECTOR_STATUS
echo 0x10007 | sudo tee $D/command && sudo cat $D/response   # GET_CONNECTOR_CAPABILITY
```

Every controller command, alert and `tcpm_reset` outcome is traced with `dev_dbg`. Turn it on with dynamic debug:

```sh
echo 'module ucsi_rts54xx +pt' | sudo tee /sys/kernel/debug/dynamic_debug/control
# e.g. "cmd 0e 03 12 00 01 -> ping 37 (0x25)": command bytes, final ping (status in bits 1:0, length in 7:2)
echo 'module ucsi_rts54xx -p' | sudo tee /sys/kernel/debug/dynamic_debug/control
```

The controller can also be probed from userspace with `i2c-tools`. Unbind the driver first so the two don't interleave transactions:

```sh
sudo i2ctransfer -f -y 0 w5@0x65 0x01 0x03 0xDA 0x0B 0x01   # VENDOR_CMD_ENABLE
sudo i2ctransfer -f -y 0 w5@0x65 0x3A 0x03 0x00 0x00 0x1F   # GET_IC_STATUS
sudo i2ctransfer -f -y 0 r1@0x65                            # ping status
sudo i2ctransfer -f -y 0 w1@0x65 0x80 r32@0x65              # block read
```

## Prior art

Realtek's own `drivers/usb/dwc3/rtk-rts5400.c` (2017, in their vendor 4.9 BSP for RTD129x set-top SoCs) talks to the same command set. It is a board-support helper, not a Type-C driver: it binds through device tree, keeps a single global device, reads status and PDOs once at probe to log them and set a 12 V GPIO, and registers nothing with the `typec` or UCSI subsystems. It has no interrupt handling, and its suspend/resume hooks are empty. It was never submitted upstream. It was, however, the reference for the vendor `GET_PDO` framing.

## License

GPL-2.0-only. See [`LICENSE`](LICENSE). The protocol is derived from the ChromiumOS EC sources (BSD-3-Clause); no code was copied from them.
