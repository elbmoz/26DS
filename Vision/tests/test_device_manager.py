import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from device_manager import (
    DeviceManagerError,
    LAUNCHER_UI_COMMAND,
    MaixCamDeviceManager,
    build_manifest,
    preflight_source,
    source_files,
)


class DeviceManagerLocalTests(unittest.TestCase):
    def _source(self, root):
        root = Path(root)
        (root / "main.py").write_text("print('ok')\n", encoding="utf-8")
        (root / "module.py").write_text("VALUE = 1\n", encoding="utf-8")
        (root / "reference.jpg").write_bytes(b"jpeg")
        (root / "__pycache__").mkdir()
        (root / "__pycache__" / "module.pyc").write_bytes(b"ignored")
        (root / "notes.txt").write_text("ignored", encoding="utf-8")
        return root

    def test_manifest_is_deterministic_and_ignores_build_artifacts(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = self._source(temporary)
            first = build_manifest(source)
            second = build_manifest(source)
            self.assertEqual(first, second)
            self.assertEqual(
                set(first["files"]),
                {"main.py", "module.py", "reference.jpg"},
            )
            self.assertEqual(len(first["release_id"]), 16)
            self.assertEqual(
                [path.name for path in source_files(source)],
                ["main.py", "module.py", "reference.jpg"],
            )

    def test_source_change_creates_a_new_release(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = self._source(temporary)
            before = build_manifest(source)["release_id"]
            (source / "module.py").write_text(
                "VALUE = 2\n", encoding="utf-8"
            )
            after = build_manifest(source)["release_id"]
            self.assertNotEqual(before, after)

    def test_preflight_compiles_every_python_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = self._source(temporary)
            checked = preflight_source(source)
            self.assertTrue(checked["ok"])
            self.assertEqual(checked["python_files_checked"], 2)

    def test_preflight_rejects_invalid_python_before_upload(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = self._source(temporary)
            (source / "module.py").write_text(
                "if broken syntax\n", encoding="utf-8"
            )
            with self.assertRaises(SyntaxError):
                preflight_source(source)

    def test_manifest_requires_main_entrypoint(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary)
            (source / "module.py").write_text("VALUE=1\n", encoding="utf-8")
            with self.assertRaises(DeviceManagerError):
                build_manifest(source)

    def test_launcher_ui_is_suspended_by_exact_pid(self):
        manager = MaixCamDeviceManager()
        launcher = {
            "pid": 321,
            "state": "S",
            "command": LAUNCHER_UI_COMMAND,
        }
        with (
            patch.object(
                manager, "_launcher_ui", return_value=launcher.copy()
            ),
            patch.object(manager, "_process_state", return_value="T"),
            patch.object(manager, "_exec") as remote_exec,
        ):
            result = manager._suspend_launcher_ui()

        remote_exec.assert_called_once_with("kill -STOP 321", timeout=5)
        self.assertTrue(result["suspended"])
        self.assertEqual(result["state"], "T")

    def test_start_refuses_when_launcher_ui_is_not_available(self):
        manager = MaixCamDeviceManager()
        with patch.object(manager, "_launcher_ui", return_value=None):
            with self.assertRaisesRegex(
                DeviceManagerError, "exit the foreground device app"
            ):
                manager._suspend_launcher_ui()

    def test_restart_keeps_launcher_suspended_during_handoff(self):
        manager = MaixCamDeviceManager()
        with (
            patch.object(
                manager, "stop", return_value={"ok": True}
            ) as stop,
            patch.object(
                manager, "start", return_value={"ok": True}
            ) as start,
        ):
            result = manager.restart(wait_seconds=2.5)

        stop.assert_called_once_with(resume_launcher=False)
        start.assert_called_once_with(wait_seconds=2.5)
        self.assertTrue(result["ok"])
