import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("updater", Path(__file__).resolve().parents[1] / "tools/update_firmware.py")
u = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u)


def entry(address="COM8", serial="ABC", vid="0x303A", pid="0x1001"):
    return {"port": {"address": address, "protocol": "serial",
                     "properties": {"vid": vid, "pid": pid, "serialNumber": serial}}}


class UpdaterTests(unittest.TestCase):
    def test_windows_mac_and_filter(self):
        ports = u.ports_from_json({"detected_ports": [entry(), entry("/dev/cu.usbmodem1", "DEF"),
                                                      entry("COM4", vid="0x1234"), entry(pid="invalid")]})
        self.assertEqual(len(ports), 2)
        self.assertEqual(u.select_port(ports, address="com8")["serial"], "ABC")
        self.assertEqual(u.select_port(ports, serial="def")["address"], "/dev/cu.usbmodem1")
        with self.assertRaises(ValueError):
            u.select_port(ports)

    def test_absent_missing_and_duplicate_serial(self):
        for ports in ([], [{"address": "COM1", "serial": ""}],
                      [{"address": "COM1", "serial": "A"}, {"address": "COM2", "serial": "A"}]):
            with self.assertRaises(ValueError):
                u.select_port(ports)

    def test_upload_does_not_erase_all(self):
        command = u.upload_command("C:/Program Files/cli.exe", "COM8", Path("space dir"))
        self.assertIn("upload.erase_cmd=", command)
        self.assertIn("upload.extra_flags=", command)
        self.assertNotIn("erase-flash", command)
        self.assertEqual(command[0], "C:/Program Files/cli.exe")

    def test_build_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "build.options.json").write_text(json.dumps({"fqbn": u.FQBN}))
            for suffix in (".bin", ".bootloader.bin"):
                (directory / (u.SKETCH + suffix)).write_bytes(b"\xe9")
            rows = [(b"nvs", 1, 2, 0x9000, 0x5000), (b"otadata", 1, 0, 0xe000, 0x2000),
                    (b"app0", 0, 0x10, 0x10000, 0x300000)]
            table = b"".join(struct.pack("<HBBII16sI", 0x50aa, kind, sub, start, size, name, 0)
                             for name, kind, sub, start, size in rows)
            path = directory / (u.SKETCH + ".partitions.bin")
            path.write_bytes(table)
            u.validate_build(directory)
            path.write_bytes(b"wrong layout")
            with self.assertRaises(ValueError):
                u.validate_build(directory)
            path.write_bytes(table)
            (directory / "build.options.json").write_text(json.dumps({"fqbn": "wrong-board"}))
            with self.assertRaises(ValueError):
                u.validate_build(directory)

    @patch.object(u, "find_cli", return_value="cli")
    @patch.object(u, "discover")
    @patch.object(u.subprocess, "run")
    def test_dry_run_never_resets_or_builds(self, run, discover, cli):
        discover.return_value = u.ports_from_json({"detected_ports": [entry()]})
        self.assertEqual(u.main(["--dry-run"]), 0)
        run.assert_not_called()

    @patch.object(u, "find_cli", return_value="cli")
    @patch.object(u, "discover")
    @patch.object(u, "validate_build")
    @patch.object(u.subprocess, "run")
    def test_reenumerated_port_follows_serial(self, run, validate, discover, cli):
        discover.side_effect = [u.ports_from_json({"detected_ports": [entry()]}),
                                u.ports_from_json({"detected_ports": [entry("COM9")]})]
        self.assertEqual(u.main(["--yes", "--input-dir", "."]), 0)
        self.assertIn("COM9", run.call_args.args[0])
        self.assertEqual(run.call_count, 1)

    @patch.object(u, "find_cli", return_value="cli")
    @patch.object(u, "discover")
    @patch.object(u, "validate_build")
    @patch.object(u.subprocess, "run")
    def test_device_swap_blocks_upload(self, run, validate, discover, cli):
        discover.side_effect = [u.ports_from_json({"detected_ports": [entry()]}),
                                u.ports_from_json({"detected_ports": [entry(serial="OTHER")]})]
        with self.assertRaises(ValueError):
            u.main(["--yes", "--input-dir", "."])
        run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
