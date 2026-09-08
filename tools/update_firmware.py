#!/usr/bin/env python3
"""Axicube USB updater. Python 3.9+, Arduino CLI + ESP32 core required."""
import argparse
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BOARD = "esp32:esp32:waveshare_esp32_s3_touch_lcd_169"
FQBN = BOARD + ":PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled"
SKETCH = "LVGL_Axicube_Replica.ino"
USB_IDS = {(0x303A, 0x1001), (0x303A, 0x821E)}


def find_cli(explicit=None):
    if explicit:
        resolved = shutil.which(explicit)
        if resolved:
            return resolved
        raise ValueError("Arduino CLI not found: " + explicit)
    found = shutil.which("arduino-cli")
    if found:
        return found
    candidates = [Path("/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"),
                  Path.home() / "Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"]
    for env, suffix in (("LOCALAPPDATA", "Programs/Arduino IDE"),
                        ("ProgramFiles", "Arduino IDE"),
                        ("ProgramFiles(x86)", "Arduino IDE")):
        if os.environ.get(env):
            candidates.append(Path(os.environ[env]) / suffix /
                              "resources/app/lib/backend/resources/arduino-cli.exe")
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise ValueError("Install Arduino CLI or Arduino IDE 2, or pass --cli PATH.")


def ports_from_json(data):
    ports = []
    for entry in data.get("detected_ports", []):
        port = entry.get("port", {})
        props = port.get("properties", {})
        try:
            usb = (int(props.get("vid", "0"), 16), int(props.get("pid", "0"), 16))
        except (ValueError, TypeError):
            continue
        if port.get("protocol") == "serial" and usb in USB_IDS and port.get("address"):
            ports.append({"address": port["address"], "usb": usb,
                          "serial": props.get("serialNumber", "")})
    return ports


def discover(cli):
    result = subprocess.run([cli, "board", "list", "--json"], check=True,
                            capture_output=True, text=True, timeout=30)
    return ports_from_json(json.loads(result.stdout))


def select_port(ports, address=None, serial=None):
    matches = [p for p in ports if (not address or p["address"].casefold() == address.casefold())
               and (not serial or p["serial"].casefold() == serial.casefold())]
    if len(matches) != 1:
        raise ValueError("Expected exactly one ESP32 USB candidate; found %d. "
                         "Use --list, then --port or --serial. No device was reset." % len(matches))
    if not matches[0]["serial"]:
        raise ValueError("USB serial number missing; cannot safely track this device.")
    return matches[0]


def validate_build(directory):
    """Refuse wrong board/layout and binaries that would overlap saved NVS."""
    options = json.loads((directory / "build.options.json").read_text(encoding="utf-8"))
    if options.get("fqbn") != FQBN:
        raise ValueError("Build target differs from the validated Waveshare 1.69 target: " + FQBN)
    bounds = {".bootloader.bin": 0x8000, ".partitions.bin": 0x1000, ".bin": 0x300000}
    for suffix, maximum in bounds.items():
        size = (directory / (SKETCH + suffix)).stat().st_size
        if not 0 < size <= maximum:
            raise ValueError("Invalid firmware size: " + suffix)
    table = (directory / (SKETCH + ".partitions.bin")).read_bytes()
    entries = {}
    for offset in range(0, len(table) - 31, 32):
        magic, kind, subtype, start, size, label, flags = struct.unpack_from("<HBBII16sI", table, offset)
        if magic != 0x50AA:
            break
        entries[label.rstrip(b"\0")] = (kind, subtype, start, size)
    expected = {b"nvs": (1, 2, 0x9000, 0x5000),
                b"otadata": (1, 0, 0xE000, 0x2000),
                b"app0": (0, 0x10, 0x10000, 0x300000)}
    if any(entries.get(name) != value for name, value in expected.items()):
        raise ValueError("Unexpected partition layout; refusing to risk calibration storage.")


def upload_command(cli, address, directory):
    return [cli, "upload", "--fqbn", FQBN, "--port", address,
            "--input-dir", str(directory), "--upload-property", "upload.erase_cmd=",
            "--upload-property", "upload.extra_flags=", str(ROOT)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", help="Arduino CLI executable path")
    parser.add_argument("--list", action="store_true", help="List USB candidates without resetting")
    parser.add_argument("--port", help="Select COM port or /dev/cu.usbmodem port")
    parser.add_argument("--serial", help="Select USB serial number (preferred for automation)")
    parser.add_argument("--input-dir", type=Path, help="Existing trusted Arduino build directory; skip compile")
    parser.add_argument("--dry-run", action="store_true", help="Discover and print plan only; no compile/reset/write")
    parser.add_argument("--yes", action="store_true", help="Confirm the selected candidate is your Waveshare 1.69")
    args = parser.parse_args(argv)
    cli = find_cli(args.cli)
    ports = discover(cli)
    for port in ports:
        print("%s  USB=%04X:%04X  serial=%s" %
              (port["address"], *port["usb"], port["serial"]), flush=True)
    if args.list:
        if not ports:
            print("No supported USB candidates found.")
        return 0
    selected = select_port(ports, args.port, args.serial)
    directory = args.input_dir.resolve() if args.input_dir else None
    if directory:
        validate_build(directory)
    print("Target:", FQBN, flush=True)
    print("USB IDs are shared by other ESP32 boards; they do NOT prove the board model.", flush=True)
    if args.dry_run:
        print("DRY RUN: %s -> recheck USB serial -> bootloader reset -> flash -> reset." %
              ("use " + str(directory) if directory else "compile current source"))
        print("No board reset or flash write performed.")
        return 0
    if not args.yes:
        if not sys.stdin.isatty():
            raise ValueError("Noninteractive update requires --yes (prefer also --serial).")
        answer = input("Confirm %s [%s] is your Waveshare 1.69. Type UPDATE: " %
                       (selected["address"], selected["serial"]))
        if answer != "UPDATE":
            print("Cancelled; no device reset.")
            return 1
    # Finish the build before touching hardware, so compile failures are harmless.
    if directory is None:
        directory = Path(tempfile.mkdtemp(prefix="axicube-firmware-"))
        print("Build artifacts (retained):", directory, flush=True)
        subprocess.run([cli, "compile", "--fqbn", FQBN, "--build-path", str(directory),
                        str(ROOT)], check=True)
        validate_build(directory)
    # Never switch to another candidate if the original device disappears.
    current = select_port(discover(cli), serial=selected["serial"])
    print("Updating", current["address"], "- keep USB connected.", flush=True)
    # ESP32 core's esptool recipe uses --before default-reset --after hard-reset.
    # No preliminary reset/1200-baud touch that could lose the current port.
    subprocess.run(upload_command(cli, current["address"], directory), check=True)
    print("Upload succeeded; uploader requested reboot. Check v0.0.2 on the screen.")
    print("Application boot is not verified by this script. Saved NVS was not erased.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError, subprocess.SubprocessError) as exc:
        print("Update failed: %s" % exc, file=sys.stderr)
        if isinstance(exc, subprocess.CalledProcessError) and exc.stderr:
            print(exc.stderr, file=sys.stderr)
        print("Close serial monitors and retry. If auto-reset fails: hold BOOT, press/release "
              "RESET, release BOOT, then rediscover with --list. After a manual bootloader "
              "entry, a manual RESET may also be needed after upload. No automatic retry "
              "or erase-all is performed.", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("Interrupted. If flashing had started, rerun the update before use.", file=sys.stderr)
        sys.exit(130)
