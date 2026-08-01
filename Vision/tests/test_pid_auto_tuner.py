import sys
import unittest
from pathlib import Path


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from pid_auto_tuner import (
    ProfileEarlyStop,
    coordinate_search,
    optuna_search,
    score_inner,
    score_outer,
    score_profile,
)


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

    def test_outer_score_counts_lost_tracking_as_invalid(self):
        samples = [
            {
                "tracking": {"valid": index % 4 != 0},
                "feedback": {
                    "feedback_version": 2,
                    "tuning_mode": 2,
                    "position_valid": 1,
                    "vision_age_ms": 20,
                    "control_error_px": 0.0,
                    "velocity_px_s": 0.0,
                    "target_rod_angle_deg": 0.0,
                    "protection_state": 0,
                },
            }
            for index in range(40)
        ]

        metrics = score_outer(samples, 5.0)

        self.assertEqual(metrics["invalid_ratio"], 0.25)

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

    def test_profile_score_combines_both_directions_and_return(self):
        samples = []
        sequence = 42
        for phase, target, start in ((1, 2.0, 0.0), (2, -2.0, 2.0)):
            for index in range(20):
                actual = target + (start - target) * (0.65 ** index)
                samples.append(
                    {
                        "feedback": {
                            "feedback_version": 3,
                            "tuning_mode": 4,
                            "tuning_sequence": sequence,
                            "tuning_phase": phase,
                            "phase_elapsed_ms": index * 20,
                            "position_valid": 1,
                            "target_rod_angle_deg": target,
                            "actual_rod_angle_deg": actual,
                            "angle_error_deg": target - actual,
                            "rod_rate_deg_s": 0.0,
                            "motor_command": 5,
                            "protection_state": 0,
                        }
                    }
                )
        for index in range(20):
            actual = -2.0 * (0.65 ** index)
            samples.append(
                {
                    "feedback": {
                        "feedback_version": 3,
                        "tuning_mode": 4,
                        "tuning_sequence": sequence,
                        "tuning_phase": 3,
                        "phase_elapsed_ms": index * 20,
                        "position_valid": 1,
                        "target_rod_angle_deg": 0.0,
                        "actual_rod_angle_deg": actual,
                        "angle_error_deg": -actual,
                        "rod_rate_deg_s": 0.0,
                        "motor_command": 5,
                        "protection_state": 0,
                    }
                }
            )

        metrics = score_profile(samples, sequence, "inner", 30.0, 5.0)

        self.assertEqual(metrics["positive"]["samples"], 20)
        self.assertEqual(metrics["negative"]["samples"], 20)
        self.assertLess(metrics["return_error"], 0.2)

    def test_profile_early_stop_detects_divergence(self):
        stop = ProfileEarlyStop("inner", 7, minimum_observation_ms=100)
        base = {
            "feedback_version": 3,
            "tuning_sequence": 7,
            "tuning_phase": 1,
            "position_valid": 1,
            "protection_state": 0,
            "angle_error_deg": 1.0,
        }
        self.assertIsNone(stop.observe({"feedback": dict(base, phase_elapsed_ms=0)}))
        diverged = dict(base, phase_elapsed_ms=120, angle_error_deg=2.0)
        self.assertEqual(
            stop.observe({"feedback": diverged}), "response_diverging"
        )

    def test_optuna_search_keeps_baseline_and_jointly_improves(self):
        result = optuna_search(
            {"inner_kp": 1.0},
            ("inner_kp",),
            lambda config, _trial: {
                "score": (config["inner_kp"] - 2.0) ** 2
            },
            trials=6,
            seed=3,
        )
        self.assertLessEqual(result["best_score"], 1.0)
        self.assertEqual(result["sampler"], "optuna.multivariate_tpe+sobol")


if __name__ == "__main__":
    unittest.main()
