import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from receiver_core import (
    FfmpegPipeline,
    SessionLogger,
    TelemetryReceiver,
    normalize_rtsp_url,
    parse_parameter_assignments,
)


class WindowsReceiverCoreTests(unittest.TestCase):
    def test_normalize_rtsp_url_replaces_wildcard_host(self):
        self.assertEqual(
            normalize_rtsp_url(
                "rtsp://0.0.0.0:8554/live", "10.16.6.1"
            ),
            "rtsp://10.16.6.1:8554/live",
        )
        self.assertEqual(
            normalize_rtsp_url(
                "rtsp://192.168.1.20:8554/live", "10.16.6.1"
            ),
            "rtsp://192.168.1.20:8554/live",
        )

    def test_parse_parameter_assignments(self):
        params = parse_parameter_assignments(
            ["target_position=0.55", "coast_frames=4"]
        )
        self.assertEqual(params["target_position"], 0.55)
        self.assertEqual(params["coast_frames"], 4)
        with self.assertRaises(ValueError):
            parse_parameter_assignments(["missing-separator"])

    def test_session_logger_writes_machine_readable_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            logger = SessionLogger(directory)
            packet = {
                "v": 1,
                "type": "tracking",
                "session": "abc",
                "seq": 1,
                "device_ms": 20,
                "frame_id": 2,
                "valid": True,
            }
            logger.log_packet(packet, "10.16.6.1", 100, 200)
            logger.log_packet(
                {
                    "v": 1,
                    "type": "stm32_feedback",
                    "session": "abc",
                    "transport_seq": 2,
                    "device_ms": 21,
                    "seq": 7,
                    "motor_status": 0,
                },
                "10.16.6.1",
                101,
                201,
            )
            logger.log_video_frame(0, packet)
            logger.write_manifest({"status": "completed"})
            logger.close()

            manifest = json.loads(
                (directory / "session.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["tracking_count"], 1)
            self.assertEqual(manifest["feedback_count"], 1)
            self.assertEqual(manifest["video_frame_count"], 1)
            self.assertEqual(
                len(
                    (directory / "tracking.csv")
                    .read_text(encoding="utf-8")
                    .strip()
                    .splitlines()
                ),
                2,
            )
            self.assertEqual(
                len(
                    (directory / "stm32_feedback.csv")
                    .read_text(encoding="utf-8")
                    .strip()
                    .splitlines()
                ),
                2,
            )

    def test_tracking_is_matched_to_video_source_time(self):
        with tempfile.TemporaryDirectory() as temporary:
            logger = SessionLogger(temporary)
            receiver = TelemetryReceiver(0, logger)
            try:
                receiver._tracking_history.extend(
                    [
                        {
                            "session": "abc",
                            "seq": 1,
                            "_host_epoch_ns": 1_000_000_000,
                        },
                        {
                            "session": "abc",
                            "seq": 2,
                            "_host_epoch_ns": 1_035_000_000,
                        },
                    ]
                )
                matched, info = receiver.tracking_for_epoch(
                    1_030_000_000
                )
                self.assertEqual(matched["seq"], 2)
                self.assertAlmostEqual(info["match_delta_ms"], 5.0)

                shifted, shifted_info = receiver.tracking_for_epoch(
                    1_000_000_000,
                    sync_offset_ms=35.0,
                )
                self.assertEqual(shifted["seq"], 2)
                self.assertAlmostEqual(
                    shifted_info["match_delta_ms"], 0.0
                )

                receiver._tracking_history.append(
                    {
                        "session": "abc",
                        "seq": 3,
                        "_host_epoch_ns": 1_070_000_000,
                    }
                )
                middle, _middle_info = receiver.tracking_for_epoch(
                    1_050_000_000
                )
                self.assertEqual(middle["seq"], 2)
                newest, _newest_info = receiver.tracking_for_epoch(
                    1_150_000_000
                )
                self.assertEqual(newest["seq"], 3)
                oldest, _oldest_info = receiver.tracking_for_epoch(
                    950_000_000
                )
                self.assertEqual(oldest["seq"], 1)
            finally:
                receiver.stop()
                logger.close()

    def test_feedback_listener_and_snapshot_are_independent(self):
        with tempfile.TemporaryDirectory() as temporary:
            logger = SessionLogger(temporary)
            receiver = TelemetryReceiver(0, logger)
            received = []
            receiver.add_feedback_listener(received.append)
            packet = {
                "type": "stm32_feedback",
                "session": "abc",
                "transport_seq": 9,
                "seq": 7,
                "motor_command": -320,
            }
            try:
                with receiver._condition:
                    receiver.latest_feedback = packet
                    listeners = tuple(receiver._feedback_listeners)
                for listener in listeners:
                    listener(dict(packet))

                self.assertEqual(received[0]["seq"], 7)
                snapshot = receiver.feedback_snapshot()
                self.assertEqual(snapshot["motor_command"], -320)
                snapshot["motor_command"] = 0
                self.assertEqual(
                    receiver.feedback_snapshot()["motor_command"],
                    -320,
                )
            finally:
                receiver.stop()
                logger.close()

    def test_ffmpeg_command_enables_low_latency_and_frame_timestamps(self):
        with tempfile.TemporaryDirectory() as temporary:
            with patch("receiver_core.shutil.which", return_value="ffmpeg"):
                pipeline = FfmpegPipeline(
                    "rtsp://10.16.6.1:8554/live",
                    448,
                    336,
                    30,
                    temporary,
                    record=True,
                )
            command = pipeline.command()
            self.assertIn("low_delay", command)
            self.assertIn("+nobuffer+discardcorrupt", command)
            self.assertIn("-copyts", command)
            self.assertIn("-avoid_negative_ts", command)
            self.assertIn("make_zero", command)
            self.assertIn("copy", command)
            self.assertNotIn("libx264", command)
            self.assertTrue(
                any("showinfo@vision_sync" in item for item in command)
            )


if __name__ == "__main__":
    unittest.main()
