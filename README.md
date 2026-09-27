# ucsi-rts54xx-dkms

A Linux UCSI transport driver, packaged for DKMS, for the **Realtek RTS54xx** USB-C/PD controllers that ASUS boards expose in ACPI as **`RTK5452`**.

Without this driver the controller has no Linux driver, and its port is missing from `/sys/class/typec/`. With it the port shows up in `typec`, with its data and power role, power mode, partner, and cable.

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
2. **Command.** Standard UCSI commands are wrapped as `0E <len> <ucsi-cmd> 00 <params…>`. Three commands need their own vendor framing: `ACK_CC_CI` (`0A 07 …`), `SET_NOTIFICATION_ENABLE` (`08 06 01 …`), and `PPM_RESET`.
3. **Completion.** The driver polls a 1-byte ping status: `cmd_sts` in bits [1:0] (BUSY / DONE / DEFERRED / ERROR) and `data_len` in bits [7:2].
4. **Response.** It then block-reads with command `0x80`; the first byte of the response is its byte count.
5. **Events.** When something changes, the chip pulls the GPIO interrupt low. The driver reads the Alert Response Address (`0x0C`), which releases the line, and reports a connector change to the UCSI core.

The driver runs each command synchronously inside `async_control` and stores the result as CCI and MESSAGE_IN, so the UCSI core sees an ordinary PPM. It reports **UCSI 1.2**, because `GET_CONNECTOR_STATUS` returns the 9-byte 1.x layout.

### Firmware quirks

- **`GET_PDOS` is rejected** with `CMD_ERROR`, whatever arguments it is given. The driver reports this as *not supported*, so source capabilities can't be listed and the kernel logs `UCSI_GET_PDOS failed (-95)` at load. This is harmless.
- **`GET_IC_STATUS`** accepts a length of at most 31 (`0x1F`), completes with `data_len = 0`, and ignores the offset byte.
- The connector-number field is ignored, because the controller has only one connector.

## Status

| | |
|---|---|
| Probe, unlock, chip identification | ✅ |
| PPM reset, capabilities, connector capability/status, cable | ✅ `/sys/class/typec/portN` with partner and cable |
| Unload/reload | ✅ attached device not disturbed |
| Hot-plug events (IRQ + ARA) | ✅ on unplug/replug the partner and cable are removed and re-created about 1 s after USB enumeration; about 5 interrupts per replug, no storm |
| Suspend/resume | ⚠️ untested |
| Source PDOs | ❌ firmware rejects `GET_PDOS` |

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

The controller can also be probed from userspace with `i2c-tools`. Unbind the driver first so the two don't interleave transactions:

```sh
sudo i2ctransfer -f -y 0 w5@0x65 0x01 0x03 0xDA 0x0B 0x01   # VENDOR_CMD_ENABLE
sudo i2ctransfer -f -y 0 w5@0x65 0x3A 0x03 0x00 0x00 0x1F   # GET_IC_STATUS
sudo i2ctransfer -f -y 0 r1@0x65                            # ping status
sudo i2ctransfer -f -y 0 w1@0x65 0x80 r32@0x65              # block read
```

## License

GPL-2.0-only. See [`LICENSE`](LICENSE). The protocol is derived from the ChromiumOS EC sources (BSD-3-Clause); no code was copied from them.
