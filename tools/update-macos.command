#!/bin/bash
set -euo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
if ! command -v python3 >/dev/null 2>&1; then
  echo "Python 3.9+ is required. Install Python, then run this script again."
  exit 1
fi
exec python3 "$script_dir/update_firmware.py" "$@"
