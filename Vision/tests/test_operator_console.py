import json
import sys
import threading
import unittest
from pathlib import Path
from urllib.request import Request, urlopen


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from operator_console import ConsoleHandler, ConsoleServer


class FakeController:
    def __init__(self):
        self.actions = []

    def state(self):
        return {
            "ok": True,
            "device": {"running": True},
            "monitor": {"state": "running"},
        }

    def frame(self):
        return b"\xff\xd8frame\xff\xd9"

    def action(self, action, body):
        self.actions.append((action, body))
        return {"ok": True, "accepted": True, "operation": action}


class OperatorConsoleHttpTests(unittest.TestCase):
    def setUp(self):
        self.server = ConsoleServer(("127.0.0.1", 0), ConsoleHandler)
        self.controller = FakeController()
        self.server.controller = self.controller
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        self.base_url = "http://127.0.0.1:{}".format(
            self.server.server_address[1]
        )

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def test_serves_dashboard_state_and_frame(self):
        with urlopen(self.base_url + "/", timeout=2) as response:
            html = response.read().decode("utf-8")
        with urlopen(self.base_url + "/app.js", timeout=2) as response:
            script = response.read().decode("utf-8")
        self.assertIn('id="metric-position"', html)
        self.assertIn('id="calibrate-left-button"', html)
        self.assertIn('id="calibrate-right-button"', html)
        self.assertIn("travel_start_px", script)
        self.assertIn("travel_end_px", script)
        self.assertIn('id="wave-grid"', html)
        self.assertIn('id="chart-picker-toggle"', html)

        with urlopen(self.base_url + "/api/state", timeout=2) as response:
            state = json.loads(response.read().decode("utf-8"))
        self.assertTrue(state["device"]["running"])

        with urlopen(
            self.base_url + "/api/frame.jpg", timeout=2
        ) as response:
            self.assertEqual(
                response.headers.get_content_type(), "image/jpeg"
            )
            self.assertEqual(response.read(), b"\xff\xd8frame\xff\xd9")

    def test_accepts_shared_machine_action_api(self):
        payload = json.dumps(
            {"action": "mark", "label": "fast"}
        ).encode("utf-8")
        request = Request(
            self.base_url + "/api/action",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=2) as response:
            result = json.loads(response.read().decode("utf-8"))
        self.assertTrue(result["accepted"])
        self.assertEqual(self.controller.actions[0][0], "mark")
        self.assertEqual(self.controller.actions[0][1]["label"], "fast")


if __name__ == "__main__":
    unittest.main()
