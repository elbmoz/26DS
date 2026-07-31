import sys
import unittest
from pathlib import Path


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from pid_auto_tuner import coordinate_search, score_inner, score_outer


def inner_sample(actual, target=2.0, mode=1):
    return {
        "feedback": {
            "feedback_version": 2,
            "tuning_mode": mode,
            "position_valid": 1,
            "target_rod_angle_deg": target,
            "actual_rod_angle_deg": actual,
            "angle_error_deg": target - actual,
            "rod_rate_deg_s": 0.0,
            "motor_command": 5,
            "protection_state": 0,
        }
    }


class PidAutoTunerTests(unittest.TestCase):
    def test_inner_score_prefers_converged_response(self):
        converged = [inner_sample(2.0 - 2.0 * (0.7 ** index)) for index in range(30)]
        stalled = [inner_sample(0.2) for _ in range(30)]
        self.assertLess(
            score_inner(converged, 30)["score"],
            score_inner(stalled, 30)["score"],
        )

    def test_outer_score_penalizes_tail_error(self):
        def rows(error):
            return [
                {
                    "feedback": {
                        "feedback_version": 2,
                        "tuning_mode": 2,
                        "position_valid": 1,
                        "vision_age_ms": 20,
                        "control_error_px": error,
                        "velocity_px_s": 0.0,
                        "target_rod_angle_deg": 1.0,
                        "protection_state": 0,
                    }
                }
                for _ in range(30)
            ]

        self.assertLess(
            score_outer(rows(2.0), 5.0)["score"],
            score_outer(rows(20.0), 5.0)["score"],
        )

    def test_coordinate_search_keeps_best_candidate(self):
        result = coordinate_search(
            {"inner_kp": 1.0},
            ("inner_kp",),
            lambda config: {
                "score": abs(config["inner_kp"] - 1.55)
            },
            rounds=1,
        )
        self.assertEqual(result["best_config"]["inner_kp"], 1.55)


if __name__ == "__main__":
    unittest.main()
