#!/usr/bin/env bash
# Explicit serial/interface mapping; no motor commands or broad tty permissions.
set -euo pipefail
serial=''; owner=''; left='01'; right='02'
while [[ $# -gt 0 ]]; do
  case "$1" in
    --serial) serial="$2"; shift 2;;
    --user) owner="$2"; shift 2;;
    --left-interface) left="$2"; shift 2;;
    --right-interface) right="$2"; shift 2;;
    *) echo 'Usage: sudo bash install_inspire_access.sh --serial SERIAL --user USER [--left-interface 01 --right-interface 02]' >&2; exit 2;;
  esac
done
[[ $EUID -eq 0 ]] || { echo 'Run this installer with sudo on the Orin.' >&2; exit 1; }
[[ "$serial" =~ ^[A-Za-z0-9_-]+$ && "$owner" =~ ^[a-z_][a-z0-9_-]*$ && "$left" =~ ^[0-9]{2}$ && "$right" =~ ^[0-9]{2}$ && "$left" != "$right" ]] || { echo 'Explicit valid serial, user, and distinct interfaces required.' >&2; exit 2; }
getent passwd "$owner" >/dev/null
getent group dialout >/dev/null
ports=()
for interface in "$left" "$right"; do
  port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_${serial}-if${interface}-port0"
  [[ -c "$port" ]] || { echo "Port not found: $port" >&2; exit 1; }
  ports+=("$port")
done
temporary=$(mktemp)
trap 'rm -f "$temporary"' EXIT
for pair in "$left:left" "$right:right"; do
  interface="${pair%%:*}"; side="${pair##*:}"
  printf 'SUBSYSTEM=="tty", ENV{ID_SERIAL_SHORT}=="%s", ENV{ID_USB_INTERFACE_NUM}=="%s", OWNER="%s", GROUP="dialout", MODE="0660", SYMLINK+="g1_inspire_%s"\n' "$serial" "$interface" "$owner" "$side" >> "$temporary"
done
install -m 0644 "$temporary" /etc/udev/rules.d/99-g1-inspire.rules
udevadm control --reload-rules
for port in "${ports[@]}"; do
  device=$(basename "$(readlink -f "$port")")
  udevadm trigger --action=change "/sys/class/tty/$device"
done
udevadm settle
stat -Lc '%U %a %n' /dev/g1_inspire_left /dev/g1_inspire_right
echo 'Persistent serial access installed; verify physical left/right mapping before teleoperation.'
