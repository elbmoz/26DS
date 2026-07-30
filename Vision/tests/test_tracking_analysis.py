import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_DIR))

from analyze_tracking_log import analyze


class TrackingAnalysisTests(unittest.TestCase):
    def test_structured_sync_metrics_are_emitted(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tracking_path = root / "tracking.csv"
            with tracking_path.open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=(
                        "frame_id",
                        "device_ms",
                        "detect_ms",
                        "loop_dt_ms",
                        "velocity_px_s",
                        "measured",
                        "valid",
                        "coasting",
                        "fps",
                        "host_monotonic_ns",
                        "position_rejects",
                        "lateral_rejects",
                        "quality_rejects",
                        "jump_rejects",
                    ),
                )
                writer.writeheader()
                for index in range(3):
                    writer.writerow(
                        {
                            "frame_id": index,
                            "device_ms": index * 33,
                            "detect_ms": 4 + index,
                            "loop_dt_ms": 33,
                            "velocity_px_s": 10,
                            "measured": True,
                            "valid": True,
                            "coasting": False,
                            "fps": 30,
                            "host_monotonic_ns": index * 33_000_000,
                            "position_rejects": index == 0,
                            "lateral_rejects": 0,
                            "quality_rejects": index == 1,
                            "jump_rejects": index == 2,
                        }
                    )

            with (root / "video_frames.csv").open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=(
                        "tracking_frame_id",
                        "video_pipeline_latency_ms",
                        "telemetry_match_delta_ms",
                        "dropped_frames",
                    ),
                )
                writer.writeheader()
                for index in range(3):
                    writer.writerow(
                        {
                            "tracking_frame_id": index,
                            "video_pipeline_latency_ms": 70 + index,
                            "telemetry_match_delta_ms": -10 + index,
                            "dropped_frames": index == 2,
                        }
                    )

            (root / "events.jsonl").write_text(
                json.dumps(
                    {
                        "host_monotonic_ns": 0,
                        "type": "experiment_marker",
                        "details": {"label": "静止"},
                    },
                    ensure_ascii=False,
                )
                + "\n",
                encoding="utf-8",
            )

            report, metrics = analyze(
                tracking_path, return_metrics=True
            )
            self.assertIn("video pipeline latency", report)
            self.assertEqual(metrics["dropped_preview_frames"], 1)
            self.assertEqual(
                metrics["video_pipeline_latency_ms"]["p50"], 71.0
            )
            self.assertEqual(
                metrics["video_telemetry_alignment_error_ms"]["p50"],
                9.0,
            )
            self.assertEqual(
                metrics["marked_stages"][0]["label"], "静止"
            )
            self.assertEqual(
                metrics["marked_stages"][0]["longest_lost_frames"], 0
            )
            self.assertTrue(
                metrics["evaluation"]["labeled_accuracy_available"]
            )
            self.assertEqual(
                metrics["candidate_rejections"]["quality_rejects"]
                ["candidates"],
                1,
            )
            self.assertIn("candidate rejects", report)

    def test_lost_rows_are_not_mislabeled_as_slow_motion(self):
        with tempfile.TemporaryDirectory() as temporary:
            tracking_path = Path(temporary) / "tracking.csv"
            with tracking_path.open(
                "w", newline="", encoding="utf-8"
            ) as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=(
                        "frame_id",
                        "device_ms",
                        "detect_ms",
                        "loop_dt_ms",
                        "velocity_px_s",
                        "measured",
                        "valid",
                        "coasting",
                        "fps",
                    ),
                )
                writer.writeheader()
                writer.writerow(
                    {
                        "frame_id": 0,
                        "device_ms": 0,
                        "detect_ms": 5,
                        "loop_dt_ms": 20,
                        "velocity_px_s": "",
                        "measured": False,
                        "valid": False,
                        "coasting": False,
                        "fps": 50,
                    }
                )
                writer.writerow(
                    {
                        "frame_id": 1,
                        "device_ms": 20,
                        "detect_ms": 5,
                        "loop_dt_ms": 20,
                        "velocity_px_s": 20,
                        "measured": True,
                        "valid": True,
                        "coasting": False,
                        "fps": 50,
                    }
                )

            report, metrics = analyze(
                tracking_path, return_metrics=True
            )

            self.assertEqual(metrics["untracked_frames"], 1)
            self.assertEqual(
                metrics["speed_groups"]["slow(<80px/s)"]["frames"], 1
            )
            self.assertIn("operational availability", report)
            self.assertFalse(
                metrics["evaluation"]["labeled_accuracy_available"]
            )


if __name__ == "__main__":
    unittest.main()
