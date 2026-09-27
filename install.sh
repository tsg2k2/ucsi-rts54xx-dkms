#!/bin/sh
# Install ucsi-rts54xx as a DKMS module and load it.
set -eu

cd "$(dirname "$0")"
NAME=ucsi-rts54xx
VER=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' src/dkms.conf)

[ "$(id -u)" -eq 0 ] || exec sudo "$0" "$@"

# Drop any other installed version first; they share the module name.
for old in $(dkms status "$NAME" 2>/dev/null | sed -n "s|^$NAME/\([^,:]*\).*|\1|p" | sort -u); do
	[ "$old" = "$VER" ] && continue
	echo "Removing $NAME/$old"
	dkms remove "$NAME/$old" --all
	rm -rf "/usr/src/$NAME-$old"
done

rm -rf "/usr/src/$NAME-$VER"
cp -r src "/usr/src/$NAME-$VER"
dkms add "$NAME/$VER" 2>/dev/null || true
dkms install "$NAME/$VER"

modprobe -r ucsi_rts54xx 2>/dev/null || true
modprobe ucsi_rts54xx

# Ports register asynchronously after probe.
ports() { ls -d /sys/bus/i2c/devices/i2c-RTK5452:*/typec/port* 2>/dev/null; }
for _ in 1 2 3 4 5 6 7 8 9 10; do
	[ -n "$(ports)" ] && break
	sleep 1
done
echo
ports | sed 's|.*/|typec |' | grep . || echo "No typec port registered (no RTS54xx controller bound?)"
