import csv
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS_DIR))

from plot_stm32_feedback import plot_feedback


class Stm32FeedbackPlotTests(unittest.TestCase):
    def test_csv_is_summarized_and_rendered_to_svg(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = directory / "stm32_feedback.csv"
            fields = (
                "mcu_ms",
                "seq",
                "seq_gap",
                "position_px",
                "velocity_px_s",
                "control_error_px",
                "p_term",
                "i_term",
                "d_term",
                "motor_command",
                "motor_status",
                "vision_age_ms",
            )
            with source.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow(
                    {
                        "mcu_ms": 100,
                        "seq": 10,
                        "seq_gap": 0,
                        "position_px": 1,
                        "velocity_px_s": 2,
                        "control_error_px": 3,
                        "p_term": 4,
                        "i_term": 5,
                        "d_term": 6,
                        "motor_command": 7,
                        "motor_status": 0,
                        "vision_age_ms": 8,
                    }
                )
                writer.writerow(
                    {
                        "mcu_ms": 120,
                        "seq": 13,
                        "seq_gap": 2,
                        "position_px": 2,
                        "velocity_px_s": 3,
                        "control_error_px": 2,
                        "p_term": 5,
                        "i_term": 6,
                        "d_term": 7,
                        "motor_command": 8,
                        "motor_status": 2,
                        "vision_age_ms": 9,
                    }
                )

            output, summary = plot_feedback(directory)
            rendered = output.read_text(encoding="utf-8")
            self.assertEqual(summary["rows"], 2)
            self.assertEqual(summary["sequence_gaps"], 2)
            self.assertEqual(summary["status_counts"][2], 1)
            self.assertIn("<svg", rendered)
            self.assertIn("STM32 control feedback", rendered)


if __name__ == "__main__":
    unittest.main()
