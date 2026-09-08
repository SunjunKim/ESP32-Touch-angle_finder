#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else "Python 3.10+ required")'
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
if ! .venv/bin/python -c 'import esptool, serial; import importlib.metadata as m; assert m.version("esptool")=="5.1.0" and m.version("pyserial")=="3.5"' >/dev/null 2>&1; then
  .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python flash_precompiled.py "$@"
