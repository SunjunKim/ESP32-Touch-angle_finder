#!/usr/bin/env python3
"""Flash a trusted Axicube binary package; never compile or invoke Arduino CLI."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

from update_firmware import FQBN, SKETCH, USB_IDS, select_port, validate_build

IMAGES = [(0x0, SKETCH + ".bootloader.bin"),
          (0x8000, SKETCH + ".partitions.bin"),
          (0xE000, "boot_app0.bin"), (0x10000, SKETCH + ".bin")]


def verify_package(directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != 1 or manifest.get("fqbn") != FQBN:
        raise ValueError("Wrong package format or board target.")
    expected = {name for _, name in IMAGES} | {"build.options.json"}
    hashes = manifest.get("sha256", {})
    if set(hashes) != expected:
        raise ValueError("Unexpected or missing package files.")
    for name in expected:
        data = (directory / name).read_bytes()
        if hashlib.sha256(data).hexdigest() != hashes[name]:
            raise ValueError("Package checksum mismatch: " + name)
    validate_build(directory)
    if not 0 < (directory / "boot_app0.bin").stat().st_size <= 0x2000:
        raise ValueError("OTA data image would overlap the app partition.")
    return manifest


def discover():
    from serial.tools import list_ports
    return [{"address": p.device, "serial": p.serial_number or "", "usb": (p.vid, p.pid)}
            for p in list_ports.comports() if (p.vid, p.pid) in USB_IDS]


def command(port, directory):
    result = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", port,
              "--baud", "460800", "--before", "default-reset", "--after", "hard-reset",
              "write-flash", "--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]
    for offset, name in IMAGES:
        result.extend([hex(offset), str(directory / name)])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware-dir", type=Path, default=Path(__file__).resolve().parent / "firmware")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--serial")
    args = parser.parse_args(argv)
    ports = discover()
    for port in ports:
        print("%s USB=%04X:%04X serial=%s" % (port["address"], *port["usb"], port["serial"]))
    if args.list:
        if not ports:
            print("No supported USB candidates found.")
        return 0
    directory = args.firmware_dir.resolve()
    manifest = verify_package(directory)
    selected = select_port(ports, args.port, args.serial)
    print("Firmware:", manifest["version"], "Target:", FQBN)
    print("USB IDs are shared. Confirm the physical board is Waveshare Touch LCD 1.69.")
    if args.dry_run:
        print("SHA-256/layout OK. DRY RUN: no reset or flash write.")
        print(subprocess.list2cmdline(command(selected["address"], directory)))
        return 0
    if importlib.metadata.version("esptool") != "5.1.0":
        raise ValueError("Install the packaged requirements.txt (esptool 5.1.0).")
    if not args.yes:
        if not sys.stdin.isatty():
            raise ValueError("Noninteractive update requires --yes; also prefer --serial.")
        if input("Type UPDATE to flash %s [%s]: " % (selected["address"], selected["serial"])) != "UPDATE":
            print("Cancelled; no reset.")
            return 1
    current = select_port(discover(), serial=selected["serial"])
    verify_package(directory)
    subprocess.run(command(current["address"], directory), check=True)
    print("Flash verified by esptool; reset requested. Check firmware %s on screen." % manifest["version"])
    print("NVS sectors were not included. Application boot is not automatically verified.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, ImportError, subprocess.SubprocessError) as exc:
        print("Update failed: %s" % exc, file=sys.stderr)
        print("Install requirements.txt, close serial monitors and check USB. If reset fails, "
              "hold BOOT, tap RESET, release BOOT, then retry --list. Manual RESET may also "
              "be needed after flashing from manual download mode. No erase-all is used.", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Interrupted. If writing had started, retry the update before use.", file=sys.stderr)
        sys.exit(130)
