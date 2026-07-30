import json
import sys
import tempfile
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import urlopen


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from iteration_bridge import IterationBridge


class IterationBridgeTests(unittest.TestCase):
    def test_status_file_http_and_command_queue(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bridge = IterationBridge(
                root / "session",
                root / "runtime",
                port=0,
            )
            bridge.start()
            try:
                bridge.publish(
                    {"state": "running", "video": {"latency_ms": 42.0}}
                )
                with urlopen(
                    "http://127.0.0.1:{}/status".format(bridge.port),
                    timeout=2,
                ) as response:
                    status = json.loads(response.read().decode("utf-8"))
                self.assertEqual(status["state"], "running")
                self.assertEqual(status["video"]["latency_ms"], 42.0)
                with self.assertRaises(HTTPError) as missing:
                    urlopen(
                        "http://127.0.0.1:{}/frame.jpg".format(
                            bridge.port
                        ),
                        timeout=2,
                    )
                self.assertEqual(missing.exception.code, 503)
                bridge.publish_frame(b"\xff\xd8test\xff\xd9")
                with urlopen(
                    "http://127.0.0.1:{}/frame.jpg".format(bridge.port),
                    timeout=2,
                ) as response:
                    self.assertEqual(
                        response.headers.get_content_type(), "image/jpeg"
                    )
                    self.assertEqual(
                        response.read(), b"\xff\xd8test\xff\xd9"
                    )
                self.assertEqual(
                    bridge.status()["preview_frame_sequence"], 1
                )
                bridge.publish_telemetry(
                    {
                        "schema": 1,
                        "tracking": {"seq": 42, "position": 0.625},
                    }
                )
                sequence, sample = bridge.wait_for_telemetry(
                    0, timeout=0.1
                )
                self.assertEqual(sequence, 1)
                self.assertEqual(sample["tracking"]["seq"], 42)
                with urlopen(
                    "http://127.0.0.1:{}/telemetry".format(
                        bridge.port
                    ),
                    timeout=2,
                ) as response:
                    self.assertEqual(
                        response.headers.get_content_type(),
                        "text/event-stream",
                    )
                    self.assertEqual(
                        response.headers.get(
                            "Access-Control-Allow-Origin"
                        ),
                        "*",
                    )
                    event_id = response.readline().decode("utf-8").strip()
                    event_data = (
                        response.readline().decode("utf-8").strip()
                    )
                self.assertEqual(event_id, "id: 1")
                self.assertTrue(event_data.startswith("data: "))
                streamed = json.loads(event_data[len("data: ") :])
                self.assertEqual(streamed["tracking"]["position"], 0.625)
                self.assertTrue(bridge.live_status_path.is_file())
                bridge.publish({"state": "running"}, force=True)
                disk_status = json.loads(
                    bridge.live_status_path.read_text(encoding="utf-8")
                )
                self.assertEqual(disk_status["state"], "running")

                queued = bridge.enqueue("mark", {"label": "fast-left"})
                commands = bridge.poll_commands()
                self.assertEqual(commands[0]["id"], queued["id"])
                self.assertEqual(commands[0]["body"]["label"], "fast-left")
                self.assertFalse(bridge.stop_requested())
                bridge.enqueue("stop")
                self.assertTrue(bridge.stop_requested())
            finally:
                bridge.stop({"state": "completed"})


if __name__ == "__main__":
    unittest.main()
