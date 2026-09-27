#!/bin/sh
# Remove every installed version of the ucsi-rts54xx DKMS module.
set -eu

NAME=ucsi-rts54xx
[ "$(id -u)" -eq 0 ] || exec sudo "$0" "$@"

modprobe -r ucsi_rts54xx 2>/dev/null || true
for v in $(dkms status "$NAME" 2>/dev/null | sed -n "s|^$NAME/\([^,:]*\).*|\1|p" | sort -u); do
	dkms remove "$NAME/$v" --all
	rm -rf "/usr/src/$NAME-$v"
done
