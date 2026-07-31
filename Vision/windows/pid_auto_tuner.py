"""Deterministic scoring and coordinate search for online PID experiments."""

import math


INNER_PARAMETER_NAMES = (
    "inner_kp",
    "inner_kd",
    "speed_limit",
    "slew",
)

OUTER_PARAMETER_NAMES = (
    "outer_kp",
    "outer_ki",
    "outer_kd",
    "angle_limit",
)

PARAMETER_BOUNDS = {
    "inner_kp": (0.1, 200.0),
    "inner_kd": (0.0, 20.0),
    "speed_limit": (5.0, 300.0),
    "slew": (0.1, 300.0),
    "outer_kp": (0.001, 0.2),
    "outer_ki": (0.0, 0.2),
    "outer_kd": (0.0, 0.1),
    "angle_limit": (0.5, 10.5),
}

SEARCH_STEPS = {
    "inner_kp": (0.65, 1.55, 1.0),
    "inner_kd": (0.55, 1.80, 0.10),
    "speed_limit": (0.70, 1.45, 20.0),
    "slew": (0.50, 2.00, 2.0),
    "outer_kp": (0.70, 1.45, 0.01),
    "outer_ki": (0.50, 1.80, 0.01),
    "outer_kd": (0.60, 1.70, 0.002),
    "angle_limit": (0.75, 1.30, 2.0),
}


def _mean(values):
    return sum(values) / float(len(values)) if values else 0.0


def _rms(values):
    return math.sqrt(_mean([value * value for value in values]))


def _tail(values, fraction=0.30):
    count = max(1, int(round(len(values) * float(fraction))))
    return values[-count:]


def _valid_v2(samples, mode):
    rows = []
    for sample in samples:
        feedback = sample.get("feedback", sample)
        if (
            feedback.get("feedback_version") == 2
            and feedback.get("tuning_mode") == mode
        ):
            rows.append(feedback)
    return rows


def score_inner(samples, speed_limit):
    rows = _valid_v2(samples, 1)
    if len(rows) < 10:
        raise ValueError("inner test has fewer than 10 F2 samples")
    valid = [row for row in rows if row.get("position_valid") == 1]
    if len(valid) < 8:
        raise ValueError("inner test has insufficient position feedback")

    errors = [float(row["angle_error_deg"]) for row in valid]
    actual = [float(row["actual_rod_angle_deg"]) for row in valid]
    rates = [float(row["rod_rate_deg_s"]) for row in valid]
    target = _mean([float(row["target_rod_angle_deg"]) for row in valid])
    start = _mean(actual[: min(5, len(actual))])
    final = _mean(_tail(actual, 0.20))
    requested_step = target - start
    response_gain = (
        (final - start) / requested_step
        if abs(requested_step) > 0.05
        else 1.0
    )
    direction = 1.0 if requested_step >= 0.0 else -1.0
    overshoot = max(
        0.0,
        max(direction * (value - target) for value in actual),
    )
    invalid_ratio = 1.0 - len(valid) / float(len(rows))
    protection_ratio = _mean(
        [1.0 if row.get("protection_state", 0) else 0.0 for row in rows]
    )
    saturation_ratio = _mean(
        [
            1.0
            if abs(float(row.get("motor_command", 0)))
            >= 0.95 * float(speed_limit)
            else 0.0
            for row in valid
        ]
    )
    rmse = _rms(errors)
    tail_rmse = _rms(_tail(errors))
    tail_rate_rms = _rms(_tail(rates))
    gain_error = abs(1.0 - response_gain)
    score = (
        rmse
        + 2.0 * tail_rmse
        + 1.5 * overshoot
        + abs(requested_step) * 1.5 * gain_error
        + 0.05 * tail_rate_rms
        + 8.0 * invalid_ratio
        + 8.0 * protection_ratio
        + 0.5 * saturation_ratio
    )
    return {
        "score": round(score, 6),
        "samples": len(rows),
        "valid_samples": len(valid),
        "rmse_deg": round(rmse, 5),
        "tail_rmse_deg": round(tail_rmse, 5),
        "overshoot_deg": round(overshoot, 5),
        "response_gain": round(response_gain, 5),
        "tail_rate_rms_deg_s": round(tail_rate_rms, 5),
        "invalid_ratio": round(invalid_ratio, 5),
        "protection_ratio": round(protection_ratio, 5),
        "saturation_ratio": round(saturation_ratio, 5),
    }


def score_outer(samples, angle_limit):
    rows = []
    tracking_valid = []
    for sample in samples:
        feedback = sample.get("feedback", sample)
        if (
            feedback.get("feedback_version") == 2
            and feedback.get("tuning_mode") == 2
        ):
            rows.append(feedback)
            tracking = sample.get("tracking")
            tracking_valid.append(
                not (
                    isinstance(tracking, dict)
                    and tracking.get("valid") is False
                )
            )
    if len(rows) < 10:
        raise ValueError("outer test has fewer than 10 F2 samples")
    valid = [
        row
        for row, vision_valid in zip(rows, tracking_valid)
        if row.get("position_valid") == 1
        and row.get("vision_age_ms", 999999) <= 200
        and vision_valid
    ]
    if len(valid) < 8:
        raise ValueError("outer test has insufficient vision/position data")

    errors = [float(row["control_error_px"]) for row in valid]
    velocities = [float(row["velocity_px_s"]) for row in valid]
    target_angles = [
        abs(float(row["target_rod_angle_deg"])) for row in valid
    ]
    invalid_ratio = 1.0 - len(valid) / float(len(rows))
    protection_ratio = _mean(
        [1.0 if row.get("protection_state", 0) else 0.0 for row in rows]
    )
    angle_saturation_ratio = _mean(
        [
            1.0 if angle >= 0.95 * float(angle_limit) else 0.0
            for angle in target_angles
        ]
    )
    rmse = _rms(errors)
    tail_rmse = _rms(_tail(errors))
    tail_velocity_rms = _rms(_tail(velocities))
    score = (
        rmse / 10.0
        + 2.0 * tail_rmse / 10.0
        + 0.03 * tail_velocity_rms
        + 10.0 * invalid_ratio
        + 10.0 * protection_ratio
        + 1.0 * angle_saturation_ratio
    )
    return {
        "score": round(score, 6),
        "samples": len(rows),
        "valid_samples": len(valid),
        "rmse_px": round(rmse, 5),
        "tail_rmse_px": round(tail_rmse, 5),
        "tail_velocity_rms_px_s": round(tail_velocity_rms, 5),
        "invalid_ratio": round(invalid_ratio, 5),
        "protection_ratio": round(protection_ratio, 5),
        "angle_saturation_ratio": round(angle_saturation_ratio, 5),
    }


def candidate_values(name, value):
    minimum, maximum = PARAMETER_BOUNDS[name]
    lower_factor, upper_factor, zero_seed = SEARCH_STEPS[name]
    if abs(float(value)) < 1e-12:
        values = (0.0, zero_seed)
    else:
        values = (float(value) * lower_factor, float(value) * upper_factor)
    result = []
    for candidate in values:
        candidate = round(max(minimum, min(maximum, candidate)), 8)
        if abs(candidate - float(value)) > 1e-10 and candidate not in result:
            result.append(candidate)
    return result


def coordinate_search(initial, names, evaluator, rounds=1):
    """Run a small deterministic coordinate search via ``evaluator(config)``."""
    current = {name: float(initial[name]) for name in names}
    trials = []

    def evaluate(config, changed):
        metrics = evaluator(dict(config))
        trial = {
            "changed": changed,
            "config": dict(config),
            "metrics": metrics,
            "score": float(metrics["score"]),
        }
        trials.append(trial)
        return trial

    best = evaluate(current, "baseline")
    for _round in range(max(1, int(rounds))):
        improved = False
        for name in names:
            parameter_best = best
            for candidate in candidate_values(name, current[name]):
                proposal = dict(current)
                proposal[name] = candidate
                trial = evaluate(proposal, name)
                if trial["score"] < parameter_best["score"]:
                    parameter_best = trial
            if parameter_best is not best:
                current = dict(parameter_best["config"])
                best = parameter_best
                improved = True
        if not improved:
            break
    return {
        "best_config": dict(current),
        "best_score": float(best["score"]),
        "trials": trials,
    }
