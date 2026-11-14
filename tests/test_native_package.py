import io
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import package_native as package


class NativePackageTests(unittest.TestCase):
    def test_macos_no_device_is_handled_without_bash_array_error(self):
        script = (package.ROOT / "tools/native-update-macos.command").read_text()
        # Simulate an empty /dev match even when the test host has USB devices.
        script = script.replace("ports=(/dev/cu.usbmodem*)", "ports=()")
        for args, status in ((["--list"], 0), ([], 1), (["--dry-run"], 1)):
            result = subprocess.run(["bash", "-s", "--", *args], input=script,
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, status, result.stderr)
            self.assertIn("No USB modem found", result.stdout)
            self.assertNotIn("unbound variable", result.stderr)
            self.assertNotIn("Type UPDATE", result.stdout)

    def test_source_archive_is_committed_and_excludes_builds(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            def git(*args):
                return subprocess.check_output(["git", *args], cwd=root, stderr=subprocess.DEVNULL)
            git("init")
            (root / ".gitignore").write_text("dist/\n")
            (root / "firmware.ino").write_text("// committed firmware\n")
            git("add", ".")
            git("-c", "user.name=Package Test", "-c", "user.email=test@example.invalid",
                "-c", "commit.gpgsign=false", "commit", "-m", "fixture")
            (root / "dist").mkdir()
            (root / "dist/private-build.bin").write_bytes(b"ignored")
            with patch.object(package, "ROOT", root):
                commit, source = package.project_source()
                self.assertEqual(commit, git("rev-parse", "HEAD").decode().strip())
                with zipfile.ZipFile(io.BytesIO(source)) as archive:
                    self.assertEqual(archive.read("LVGL_Axicube_Replica/firmware.ino"),
                                     b"// committed firmware\n")
                    self.assertFalse(any("dist/" in name or ".git/" in name for name in archive.namelist()))
                (root / "firmware.ino").write_text("// uncommitted firmware\n")
                with self.assertRaisesRegex(ValueError, "Commit project changes"):
                    package.project_source()
                git("checkout", "--", "firmware.ino")
                (root / "untracked.h").write_text("// untracked dependency\n")
                with self.assertRaisesRegex(ValueError, "Commit project changes"):
                    package.project_source()

    def test_cached_download_hash(self):
        with tempfile.TemporaryDirectory() as temp:
            file = Path(temp) / "asset"
            file.write_bytes(b"official bytes")
            self.assertEqual(package.fetch("unused", file, package.digest(b"official bytes")), b"official bytes")
            with self.assertRaises(ValueError):
                package.fetch("unused", file, "wrong")

    def test_named_archive_member_only(self):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, "w") as archive:
            archive.writestr("tool/esptool.exe", b"binary")
            archive.writestr("../../unwanted", b"never extracted")
        self.assertEqual(package.archive_member(stream.getvalue(), "zip", "esptool.exe"), b"binary")
        with self.assertRaises(ValueError):
            package.archive_member(stream.getvalue(), "zip", "missing")

    def test_duplicate_executable_rejected(self):
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, "w") as archive:
            archive.writestr("a/esptool.exe", b"a")
            archive.writestr("b/esptool.exe", b"b")
        with self.assertRaises(ValueError):
            package.archive_member(stream.getvalue(), "zip", "esptool.exe")

    def test_native_scripts_have_no_python_or_erase_all(self):
        for name in ("native-update-macos.command", "native-update-windows.ps1", "native-update-windows.cmd"):
            text = (package.ROOT / "tools" / name).read_text().lower()
            for forbidden in ("python", "pip install", "erase-flash", "erase-all"):
                self.assertNotIn(forbidden, text)
        for name in ("native-update-macos.command", "native-update-windows.ps1"):
            text = (package.ROOT / "tools" / name).read_text().lower()
            self.assertIn("default-reset", text)
            self.assertIn("hard-reset", text)
            for offset in ("0x8000", "0xe000", "0x10000"):
                self.assertIn(offset, text)
            self.assertNotIn("0x9000", text)


if __name__ == "__main__":
    unittest.main()
