#!/usr/bin/env bash
# List USB serial ports with the IDs needed for scripts/99-mc4.rules.
# Plug in the ESP32 and the RPLIDAR A1, then run:  bash scripts/list_serial.sh
shopt -s nullglob
ports=(/dev/ttyUSB* /dev/ttyACM*)
if [ ${#ports[@]} -eq 0 ]; then
  echo "No USB serial ports found. Check cables (and usbipd attach if on WSL2)."
  exit 1
fi
for dev in "${ports[@]}"; do
  info=$(udevadm info -q property -n "$dev")
  get() { echo "$info" | sed -n "s/^$1=//p"; }
  printf '%-14s vendor=%s product=%s serial=%s  (%s)\n' \
    "$dev" "$(get ID_VENDOR_ID)" "$(get ID_MODEL_ID)" "$(get ID_SERIAL_SHORT)" "$(get ID_MODEL)"
done
echo
echo "10c4:ea60 = CP2102 (RPLIDAR A1 adapter, some ESP32 boards)"
echo "1a86:7523 = CH340, 1a86:55d4 = CH9102 (most other ESP32 boards)"
