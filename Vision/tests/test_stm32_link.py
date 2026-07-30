import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from stm32_link import Stm32Link, format_stm32_line


class FakeSerial:
    def __init__(self):
        self.rx_chunks = []
        self.tx_lines = []

    def read(self):
        if not self.rx_chunks:
            return b""
        return self.rx_chunks.pop(0)

    def write_str(self, line):
        self.tx_lines.append(line)
        return len(line)


class Stm32LinkTests(unittest.TestCase):
    def test_frame_contains_position_error_and_axis_velocity(self):
        line = format_stm32_line(
            {
                "valid": True,
                "error_px": -12,
                "velocity_px_s": 18.4,
            },
            output_scale=4.0 / 3.0,
        )
        self.assertEqual(line, "B,-16,25\n")
        self.assertEqual(format_stm32_line({"valid": False}), "none\n")

    def test_stream_starts_only_after_c2_and_stops_on_ok(self):
        serial = FakeSerial()
        link = Stm32Link(serial)
        state = {
            "valid": True,
            "error_px": 20,
            "velocity_px_s": -30,
        }

        self.assertEqual(link.send_state(state), 0)
        self.assertEqual(serial.tx_lines, [])

        serial.rx_chunks.extend([b"c", b"2"])
        link.poll_commands()
        link.poll_commands()
        self.assertTrue(link.streaming)
        link.send_state(state)
        self.assertEqual(serial.tx_lines, ["B,20,-30\n"])

        serial.rx_chunks.append(b"ok")
        link.poll_commands()
        self.assertFalse(link.streaming)
        self.assertEqual(link.send_state(state), 0)
        self.assertEqual(serial.tx_lines, ["B,20,-30\n"])

    def test_multiple_commands_are_processed_in_order(self):
        serial = FakeSerial()
        serial.rx_chunks.append(b"\r\nc2okc2\n")
        link = Stm32Link(serial)
        link.poll_commands()
        self.assertTrue(link.streaming)
        self.assertEqual(link.start_count, 2)
        self.assertEqual(link.stop_count, 1)


if __name__ == "__main__":
    unittest.main()
