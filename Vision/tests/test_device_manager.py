import sys
import tempfile
import unittest
from pathlib import Path


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from device_manager import (
    DeviceManagerError,
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
