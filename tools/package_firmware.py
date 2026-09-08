#!/usr/bin/env python3
"""Developer tool: compile (or reuse a trusted build) and create a portable ZIP."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile

from update_firmware import ROOT, FQBN, SKETCH, find_cli, validate_build
from flash_precompiled import IMAGES, verify_package


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, help="Reuse an existing trusted, current-source build")
    parser.add_argument("--output", type=Path, default=ROOT / "dist")
    parser.add_argument("--cli")
    args = parser.parse_args()
    version = re.search(r'kAppVersion\[\]\s*=\s*"v([0-9]+\.[0-9]+\.[0-9]+)"',
                        (ROOT / SKETCH).read_text(encoding="utf-8")).group(1)
    target = args.output.resolve() / ("axicube-" + version)
    archive = target.with_suffix(target.suffix + ".zip")
    if target.exists() or archive.exists():
        raise ValueError("Output already exists; select a new --output directory.")
    build = args.build_dir.resolve() if args.build_dir else Path(tempfile.mkdtemp(prefix="axicube-release-"))
    if not args.build_dir:
        print("Build directory:", build, flush=True)
        subprocess.run([find_cli(args.cli), "compile", "--fqbn", FQBN, "--build-path", str(build), str(ROOT)], check=True)
    validate_build(build)
    if ("v" + version).encode() not in (build / (SKETCH + ".bin")).read_bytes():
        raise ValueError("Binary does not contain the expected version; rebuild current source.")
    target.mkdir(parents=True)
    firmware = target / "firmware"
    firmware.mkdir()
    for _, name in IMAGES:
        shutil.copyfile(build / name, firmware / name)
    # Do not distribute developer home paths, toolchain paths or build caches.
    (firmware / "build.options.json").write_text(json.dumps({"fqbn": FQBN}, indent=2) + "\n", encoding="utf-8")
    names = [name for _, name in IMAGES] + ["build.options.json"]
    manifest = {"schema": 1, "version": version, "fqbn": FQBN,
                "sha256": {name: hashlib.sha256((firmware / name).read_bytes()).hexdigest() for name in names}}
    (firmware / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    verify_package(firmware)
    for name in ("flash_precompiled.py", "update_firmware.py", "flash-macos.command", "flash-windows.cmd"):
        shutil.copyfile(ROOT / "tools" / name, target / name)
    shutil.copyfile(ROOT / "tools/requirements-flash.txt", target / "requirements.txt")
    shutil.copyfile(ROOT / "tools/PRECOMPILED.md", target / "README.md")
    (target / "flash-macos.command").chmod(0o755)
    with zipfile.ZipFile(archive, "x", zipfile.ZIP_DEFLATED) as out:
        for path in sorted(target.rglob("*")):
            if path.is_file():
                out.write(path, path.relative_to(target.parent))
    print("Package:", archive)
    print("ZIP SHA-256:", hashlib.sha256(archive.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
