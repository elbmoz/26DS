import sys
import unittest
from pathlib import Path


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from stream_receiver import (
    _StopRequested,
    _read_latest_frame,
    _subscribe_until_ack,
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


class _RetryReceiver:
    def __init__(self):
        self.subscriptions = 0
        self.waits = 0

    def subscribe(self, _device_ip, _control_port, _token):
        self.subscriptions += 1
        return "request-{}".format(self.subscriptions)

    def wait_for_ack(self, _request_id, timeout):
        self.waits += 1
        self.last_timeout = timeout
        if self.waits < 2:
            return None
        return {"ok": True}


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

    def test_subscription_retries_until_device_acknowledges(self):
        receiver = _RetryReceiver()

        ack = _subscribe_until_ack(
            receiver,
            "10.16.6.1",
            42102,
            "token",
            timeout=2.0,
            bridge=None,
            retry_interval=0.01,
        )

        self.assertTrue(ack["ok"])
        self.assertEqual(receiver.subscriptions, 2)
        self.assertEqual(receiver.waits, 2)


if __name__ == "__main__":
    unittest.main()
