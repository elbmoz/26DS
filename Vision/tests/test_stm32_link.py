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
    parse_pid_ack_line,
    parse_q9_line,
    q9_overlay_lines,
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
    Q9_LINE = "Q9,17,25340,4897,-123,48,906,1,1,0,84,-1,0"

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

    def test_f2_feedback_exposes_complete_cascade_state(self):
        feedback = parse_stm32_feedback_line(
            "F2,4,100,2,20,15,-25,35,120,0,-40,200,150,-10,"
            "50,-250,12,5,1,0,0,1"
        )
        self.assertEqual(feedback["feedback_version"], 2)
        self.assertEqual(feedback["target_rod_angle_deg"], 2.0)
        self.assertEqual(feedback["actual_rod_angle_deg"], 1.5)
        self.assertEqual(feedback["angle_error_deg"], 0.5)
        self.assertEqual(feedback["desired_motor_speed"], -2.5)
        self.assertEqual(feedback["tuning_mode"], 1)

    def test_pid_requests_use_mask_and_ack_updates_snapshot(self):
        serial = FakeSerial()
        serial.rx_chunks.append(b"c2")
        link = Stm32Link(serial)
        link.poll_commands()

        sequence = link.send_pid_request(
            "set", {"inner_kp": 3.5, "speed_limit": 30}
        )
        self.assertEqual(sequence, 1)
        self.assertEqual(serial.tx_lines[-1], "PS,1,40,3.5,30\n")

        ack = parse_pid_ack_line(
            "PA,1,0,26044,4600,10500,3500,100,3000,15000,"
            "30,50,0,0,0"
        )
        self.assertTrue(ack["ok"])
        self.assertEqual(ack["config"]["inner_kp"], 3.5)
        self.assertEqual(ack["config"]["speed_limit"], 30.0)

    def test_successful_pid_ack_restores_stream_after_maix_restart(self):
        serial = FakeSerial()
        link = Stm32Link(serial)
        link.send_pid_request("get")
        serial.rx_chunks.append(
            b"PA,1,0,26044,4600,10500,3500,0,2400,15000,"
            b"30,50,0,0,0\n"
        )

        link.poll_commands()

        self.assertTrue(link.streaming)
        self.assertEqual(len(link.drain_pid_acks()), 1)

    def test_q9_parser_exposes_raw_and_scaled_angles(self):
        frame = parse_q9_line(self.Q9_LINE)
        self.assertEqual(frame["seq"], 17)
        self.assertEqual(frame["mcu_ms"], 25340)
        self.assertEqual(frame["motor_position"], 4897)
        self.assertEqual(frame["angle_x_x10"], -123)
        self.assertEqual(frame["angle_y_x10"], 48)
        self.assertEqual(frame["angle_z_x10"], 906)
        self.assertEqual(frame["angle_x_deg"], -12.3)
        self.assertEqual(frame["angle_y_deg"], 4.8)
        self.assertEqual(frame["angle_z_deg"], 90.6)
        self.assertEqual(frame["move_direction"], -1)

    def test_q9_parser_strictly_validates_fields(self):
        fields = self.Q9_LINE.split(",")
        invalid_changes = (
            (0, "Q8"),
            (1, "1.0"),
            (7, "2"),
            (8, "-1"),
            (9, "4"),
            (11, "2"),
            (12, "-1"),
        )
        for index, value in invalid_changes:
            changed = list(fields)
            changed[index] = value
            with self.subTest(index=index, value=value):
                with self.assertRaises(ValueError):
                    parse_q9_line(",".join(changed))

        with self.assertRaises(ValueError):
            parse_q9_line(",".join(fields[:-1]))
        with self.assertRaises(ValueError):
            parse_q9_line(self.Q9_LINE + ",0")
        with self.assertRaises(ValueError):
            parse_q9_line("x" + self.Q9_LINE)
        with self.assertRaises(ValueError):
            parse_q9_line(self.Q9_LINE.encode("ascii") + b"\xff")

    def test_q9_split_join_and_commands_share_one_uart_stream(self):
        serial = FakeSerial()
        serial.rx_chunks.extend(
            [
                b"c",
                b"2Q9,17,25340,4897,-123,48,",
                b"906,1,1,0,84,-1,0\r",
                b"\nQ9,18,25540,4901,-120,50,900,1,1,0,85,1,0\nok",
            ]
        )
        link = Stm32Link(serial)
        for _ in range(4):
            link.poll_commands()

        self.assertFalse(link.streaming)
        self.assertEqual(link.start_count, 1)
        self.assertEqual(link.stop_count, 1)
        self.assertEqual(link.q9_frame_count, 2)
        self.assertEqual(link.q9_parse_error_count, 0)
        self.assertEqual(link.get_latest_q9()["seq"], 18)

    def test_q9_never_enables_question_2_streaming(self):
        serial = FakeSerial()
        serial.rx_chunks.append((self.Q9_LINE + "\n").encode("ascii"))
        link = Stm32Link(serial)
        link.poll_commands()

        self.assertFalse(link.streaming)
        self.assertEqual(link.start_count, 0)
        self.assertEqual(link.send_state({"valid": False}), 0)
        self.assertEqual(serial.tx_lines, [])

    def test_q9_and_question_2_feedback_can_arrive_together(self):
        serial = FakeSerial()
        serial.rx_chunks.append(
            (
                self.Q9_LINE
                + "\nF,10,1,1,1,0,0,0,0,0,0,0,0\n"
                + self.Q9_LINE.replace("Q9,17", "Q9,18")
                + "\n"
            ).encode("ascii")
        )
        link = Stm32Link(serial)
        link.poll_commands()

        self.assertEqual(link.q9_frame_count, 2)
        self.assertEqual(link.feedback_count, 1)
        self.assertEqual(link.drain_feedback()[0]["seq"], 10)

    def test_bad_q9_frames_do_not_replace_last_valid_frame(self):
        serial = FakeSerial()
        serial.rx_chunks.extend(
            [
                (self.Q9_LINE + "\n").encode("ascii"),
                b"Q9,18,1,2,3,4,\xff,1,1,0,1,0,0\n",
                b"Q9,broken\n",
                b"Q9," + b"1" * 600 + b"\n",
            ]
        )
        link = Stm32Link(serial)
        for _ in range(4):
            link.poll_commands()

        self.assertEqual(link.get_latest_q9()["seq"], 17)
        self.assertEqual(link.q9_frame_count, 1)
        self.assertEqual(link.q9_parse_error_count, 3)
        self.assertGreaterEqual(link.rx_error_count, 2)
        self.assertLessEqual(len(link.rx_buffer), 512)

    def test_q9_sequence_gap_uses_unsigned_wrap_and_ignores_repeats(self):
        serial = FakeSerial()

        def q9(sequence):
            return (
                "Q9,{},1,2,3,4,5,1,1,0,6,0,0\n".format(sequence)
            ).encode("ascii")

        serial.rx_chunks.extend(
            [
                q9(0xFFFFFFFE),
                q9(1),
                q9(1),
                q9(0),
                q9(2),
            ]
        )
        link = Stm32Link(serial)
        gaps = []
        for _ in range(5):
            link.poll_commands()
            gaps.append(link.get_latest_q9()["seq_gap"])

        self.assertEqual(gaps, [0, 2, 0, 0, 0])
        self.assertEqual(link.q9_sequence_gap_count, 2)
        self.assertEqual(link.last_q9_seq, 2)

    def test_q9_snapshot_is_copied_and_overlay_is_compact(self):
        link = Stm32Link(FakeSerial())
        self.assertIsNone(link.get_latest_q9())
        frame = parse_q9_line(self.Q9_LINE)
        link._queue_q9(frame)

        snapshot = link.get_latest_q9()
        snapshot["motor_position"] = 0
        self.assertEqual(link.get_latest_q9()["motor_position"], 4897)
        self.assertEqual(
            q9_overlay_lines(link.get_latest_q9()),
            (
                "Q9 P:4897",
                "X:-12.3 Y:4.8 Z:90.6",
                "IMU:V POS:V RX:0 N:84",
                "DIR:-1 MOVE:0",
            ),
        )


if __name__ == "__main__":
    unittest.main()
