import sys
import unittest
from pathlib import Path


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from stream_receiver import (
    _StopRequested,
    _read_latest_frame,
    _wait_for_status,
)


class _StoppedBridge:
    def stop_requested(self):
        return True


class _UnusedReceiver:
    def wait_for_status(self, _timeout):
        raise AssertionError("stop must be checked before waiting")


class _UnusedPipeline:
    returncode = None
    reader_error = None

    def read_latest_frame(self, _timeout):
        raise AssertionError("stop must be checked before reading")


class StreamReceiverHelperTests(unittest.TestCase):
    def test_startup_wait_honors_stop_before_telemetry(self):
        with self.assertRaises(_StopRequested):
            _wait_for_status(
                _UnusedReceiver(),
                timeout=15.0,
                bridge=_StoppedBridge(),
            )

    def test_frame_wait_honors_stop_before_first_frame(self):
        with self.assertRaises(_StopRequested):
            _read_latest_frame(
                _UnusedPipeline(),
                timeout=15.0,
                bridge=_StoppedBridge(),
            )


if __name__ == "__main__":
    unittest.main()
