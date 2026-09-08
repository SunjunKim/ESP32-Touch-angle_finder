#!/usr/bin/env python3
"""Developer-only packager. End users do not need Python or network access."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import re
import shutil
import tarfile
import urllib.request
import zipfile

from update_firmware import ROOT, SKETCH, validate_build

BASE = "https://github.com/espressif/esptool/releases/download/v5.1.0/"
ASSETS = {
    "macos-arm64": ("tar.gz", "5d5aab5b64b10dc5001cfa96b5bfa48393ae561e6d797c41a1fdd3f5d3843d03"),
    "macos-amd64": ("tar.gz", "c485511e0906cb1e0277c5eecff1c4a77b89d76d0c940b685dc9fce2fad4b242"),
    "windows-amd64": ("zip", "f68a8f7728adfc59cd60f9424928199e76eac66372c7bdc23898aa32753a437a"),
}
SOURCE_URL = "https://files.pythonhosted.org/packages/c2/03/d7d79a77dd787dbe6029809c5f81ad88912340a131c88075189f40df3aba/esptool-5.1.0.tar.gz"
SOURCE_SHA = "2ea9bcd7eb263d380a4fe0170856a10e4c65e3f38c757ebdc73584c8dd8322da"
IMAGES = {"app.bin": SKETCH + ".bin", "bootloader.bin": SKETCH + ".bootloader.bin",
          "partitions.bin": SKETCH + ".partitions.bin", "boot_app0.bin": "boot_app0.bin"}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def fetch(url, destination, expected):
    if destination.exists():
        data = destination.read_bytes()
    else:
        print("Downloading", url, flush=True)
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        if digest(data) != expected:
            raise ValueError("Upstream checksum mismatch: " + url)
        destination.write_bytes(data)
    if digest(data) != expected:
        raise ValueError("Cached checksum mismatch: " + str(destination))
    return data


def archive_member(data, extension, basename):
    # Read only a regular, named member; never extract arbitrary archive paths.
    if extension == "zip":
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            matches = [n for n in archive.namelist() if Path(n).name == basename]
            if len(matches) != 1:
                raise ValueError("Expected one upstream " + basename)
            return archive.read(matches[0])
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        matches = [m for m in archive.getmembers() if m.isfile() and Path(m.name).name == basename]
        if len(matches) != 1:
            raise ValueError("Expected one upstream " + basename)
        return archive.extractfile(matches[0]).read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    args = parser.parse_args()
    build = args.build_dir.resolve()
    validate_build(build)
    version = re.search(r'kAppVersion\[\]\s*=\s*"v([0-9]+\.[0-9]+\.[0-9]+)"',
                        (ROOT / SKETCH).read_text()).group(1)
    if ("v" + version).encode() not in (build / IMAGES["app.bin"]).read_bytes():
        raise ValueError("Firmware version mismatch; compile current source first.")
    if not 0 < (build / "boot_app0.bin").stat().st_size <= 0x2000:
        raise ValueError("Invalid OTA initializer size.")
    targets = {platform: args.output.resolve() / ("axicube-" + version + "-" + platform) for platform in ASSETS}
    if any(p.exists() or Path(str(p) + ".zip").exists() for p in targets.values()):
        raise ValueError("Output exists; select a new --output directory.")
    args.cache.mkdir(parents=True, exist_ok=True)
    source = fetch(SOURCE_URL, args.cache / "esptool-5.1.0-source.tar.gz", SOURCE_SHA)
    for platform, (extension, checksum) in ASSETS.items():
        name = "esptool-v5.1.0-" + platform + "." + extension
        data = fetch(BASE + name, args.cache / (platform + "." + extension), checksum)
        target = targets[platform]
        (target / "tool").mkdir(parents=True)
        (target / "firmware").mkdir()
        (target / "source").mkdir()
        executable = "esptool.exe" if platform.startswith("windows") else "esptool"
        (target / "tool" / executable).write_bytes(archive_member(data, extension, executable))
        (target / "tool" / executable).chmod(0o755)
        for extra in ("LICENSE", "README.md"):
            (target / "tool" / extra).write_bytes(archive_member(data, extension, extra))
        for destination, original in IMAGES.items():
            shutil.copyfile(build / original, target / "firmware" / destination)
        (target / "source/esptool-5.1.0.tar.gz").write_bytes(source)
        (target / "VERSION").write_text(version + "\n")
        if platform.startswith("windows"):
            shutil.copyfile(ROOT / "tools/native-update-windows.cmd", target / "update.cmd")
            shutil.copyfile(ROOT / "tools/native-update-windows.ps1", target / "update.ps1")
        else:
            shutil.copyfile(ROOT / "tools/native-update-macos.command", target / "update.command")
            (target / "update.command").chmod(0o755)
        shutil.copyfile(ROOT / "tools/NATIVE_RELEASE.md", target / "README.md")
        (target / "UPSTREAM.json").write_text(json.dumps({"url": BASE + name, "sha256": checksum,
                                                        "source_url": SOURCE_URL, "source_sha256": SOURCE_SHA}, indent=2) + "\n")
        checked = ["tool/" + executable] + ["firmware/" + name for name in IMAGES]
        (target / "SHA256SUMS").write_text("".join(digest((target / name).read_bytes()) + "  " + name + "\n" for name in checked))
        output = Path(str(target) + ".zip")
        with zipfile.ZipFile(output, "x", zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(target.rglob("*")):
                if path.is_file():
                    archive.write(path, path.relative_to(target.parent))
        print("Created", output, "SHA256", digest(output.read_bytes()), flush=True)


if __name__ == "__main__":
    main()
