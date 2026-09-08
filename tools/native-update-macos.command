#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
port=""; dry=0; yes=0; list=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --port) [ "$#" -ge 2 ] || exit 2; port="$2"; shift 2 ;;
    --dry-run) dry=1; shift ;;
    --yes) yes=1; shift ;;
    --list) list=1; shift ;;
    *) echo "Usage: $0 [--list] [--dry-run] [--port /dev/cu.usbmodem...] [--yes]"; exit 2 ;;
  esac
done
shopt -s nullglob
ports=(/dev/cu.usbmodem*)
printf 'USB modem candidates (model NOT verified):\n'
for candidate in "${ports[@]}"; do printf '  %s\n' "$candidate"; done
[ "$list" -eq 0 ] || exit 0
if [ -z "$port" ]; then
  [ "${#ports[@]}" -eq 1 ] || { echo "Select exactly one candidate using --port."; exit 1; }
  port="${ports[0]}"
fi
found=0
for candidate in "${ports[@]}"; do [ "$port" != "$candidate" ] || found=1; done
[ "$found" -eq 1 ] || { echo "Selected port is not a connected USB modem."; exit 1; }
identity=$(stat -f '%i:%r' "$port")
# Fixed paths only. The checksum file is integrity metadata, not a signature.
for file in tool/esptool firmware/app.bin firmware/bootloader.bin firmware/partitions.bin firmware/boot_app0.bin; do
  expected=$(awk -v name="$file" '$2==name {print $1}' SHA256SUMS)
  actual=$(shasum -a 256 "$file" | awk '{print $1}')
  [ -n "$expected" ] && [ "$expected" = "$actual" ] || { echo "Checksum failed: $file"; exit 1; }
done
echo "Firmware: $(cat VERSION). Confirm the physical device is Waveshare S3 Touch LCD 1.69."
if [ "$dry" -eq 1 ]; then echo "DRY RUN: checksums OK; no reset or write."; exit 0; fi
if [ "$yes" -eq 0 ]; then
  read -r -p "Type UPDATE to flash $port: " answer
  [ "$answer" = UPDATE ] || { echo "Cancelled."; exit 1; }
fi
[ "$identity" = "$(stat -f '%i:%r' "$port")" ] || { echo "USB port changed; rediscover."; exit 1; }
if ./tool/esptool --chip esp32s3 --port "$port" --baud 460800 --before default-reset --after hard-reset \
  write-flash --flash-mode keep --flash-freq keep --flash-size keep \
  0x0 firmware/bootloader.bin 0x8000 firmware/partitions.bin \
  0xe000 firmware/boot_app0.bin 0x10000 firmware/app.bin; then
  echo "Upload complete; reset requested. Check version, touch and CAL on screen."
else
  echo "Upload failed. Close serial monitors. Try BOOT held + RESET, release BOOT, then rediscover."
  exit 1
fi
