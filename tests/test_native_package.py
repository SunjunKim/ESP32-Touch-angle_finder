import io
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import package_native as package


class NativePackageTests(unittest.TestCase):
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
