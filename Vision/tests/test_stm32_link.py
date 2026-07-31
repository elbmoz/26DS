import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from stm32_link import (
    Stm32Link,
    feedback_csv_header,
    feedback_csv_row,
    format_stm32_line,
    parse_stm32_feedback_line,
)


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

    def test_feedback_is_parsed_scaled_and_queued_across_reads(self):
        serial = FakeSerial()
        serial.rx_chunks.extend(
            [
                b"c2F,40,1234,88,7,15,-25,35,120,-20,5,",
                b"-320,2\n",
            ]
        )
        link = Stm32Link(serial)
        link.poll_commands()
        link.poll_commands()

        self.assertTrue(link.streaming)
        feedback = link.drain_feedback()
        self.assertEqual(len(feedback), 1)
        self.assertEqual(feedback[0]["seq"], 40)
        self.assertEqual(feedback[0]["position_px"], 1.5)
        self.assertEqual(feedback[0]["velocity_px_s"], -2.5)
        self.assertEqual(feedback[0]["control_error_px"], 3.5)
        self.assertEqual(feedback[0]["p_term"], 1.2)
        self.assertEqual(feedback[0]["motor_status_name"], "HAL_BUSY")
        self.assertEqual(link.feedback_error_count, 0)

    def test_feedback_sequence_gaps_and_malformed_lines_are_counted(self):
        serial = FakeSerial()
        serial.rx_chunks.append(
            b"F,10,1,1,1,0,0,0,0,0,0,0,0\n"
            b"F,broken\n"
            b"F,13,2,2,2,0,0,0,0,0,0,0,1\n"
        )
        link = Stm32Link(serial)
        link.poll_commands()
        feedback = link.drain_feedback()

        self.assertEqual([item["seq_gap"] for item in feedback], [0, 2])
        self.assertEqual(link.feedback_gap_count, 2)
        self.assertEqual(link.feedback_error_count, 1)

    def test_feedback_csv_preserves_raw_and_scaled_fields(self):
        feedback = parse_stm32_feedback_line(
            "F,1,20,3,4,15,25,-35,125,-50,0,200,0"
        )
        feedback["seq_gap"] = 0
        header = feedback_csv_header()
        row = feedback_csv_row(feedback, 99)
        self.assertIn("position_x10", header)
        self.assertIn("position_px", header)
        self.assertIn("F,1,20,3,4", row)
        self.assertTrue(row.startswith("99,1,0,20,3,4,"))


if __name__ == "__main__":
    unittest.main()
