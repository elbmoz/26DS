"""Scoring, early pruning and joint search for online PID experiments."""

import math
import warnings


PROFILE_MODES = {"inner": 4, "outer": 5}
PROFILE_PREPARE = 0
PROFILE_POSITIVE = 1
PROFILE_NEGATIVE = 2
PROFILE_RETURN = 3
PROFILE_DONE = 4
PROFILE_ABORTED = 5


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
            feedback.get("feedback_version", 0) >= 2
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


def _profile_samples(samples, sequence, mode, phase=None):
    rows = []
    expected_mode = PROFILE_MODES[mode]
    for sample in samples:
        feedback = sample.get("feedback", sample)
        if feedback.get("feedback_version") != 3:
            continue
        if int(feedback.get("tuning_sequence", -1)) != int(sequence):
            continue
        current_phase = int(feedback.get("tuning_phase", -1))
        if phase is not None and current_phase != int(phase):
            continue
        if current_phase not in (PROFILE_DONE, PROFILE_ABORTED) and int(
            feedback.get("tuning_mode", -1)
        ) != expected_mode:
            continue
        rows.append(sample)
    return rows


def _legacy_segment(samples, mode):
    """Present one F3 phase to the existing constant-target scorers."""
    converted = []
    legacy_mode = 1 if mode == "inner" else 2
    for sample in samples:
        value = dict(sample)
        feedback = dict(sample.get("feedback", sample))
        feedback["feedback_version"] = 2
        feedback["tuning_mode"] = legacy_mode
        value["feedback"] = feedback
        converted.append(value)
    return converted


def score_profile(samples, sequence, stage, speed_limit, angle_limit):
    """Score one MCU-timed zero/+/-/zero profile as a single experiment."""
    positive = _profile_samples(
        samples, sequence, stage, PROFILE_POSITIVE
    )
    negative = _profile_samples(
        samples, sequence, stage, PROFILE_NEGATIVE
    )
    returned = _profile_samples(samples, sequence, stage, PROFILE_RETURN)
    if not positive or not negative or not returned:
        raise ValueError("profile is missing positive, negative or return data")

    if stage == "inner":
        positive_metrics = score_inner(
            _legacy_segment(positive, stage), speed_limit
        )
        negative_metrics = score_inner(
            _legacy_segment(negative, stage), speed_limit
        )
        return_rows = [
            sample.get("feedback", sample) for sample in returned
        ]
        return_tail = _tail(return_rows, 0.30)
        return_error = _rms(
            [float(row["actual_rod_angle_deg"]) for row in return_tail]
        )
        return_rate = _rms(
            [float(row["rod_rate_deg_s"]) for row in return_tail]
        )
        return_penalty = 2.0 * return_error + 0.03 * return_rate
    else:
        positive_metrics = score_outer(
            _legacy_segment(positive, stage), angle_limit
        )
        negative_metrics = score_outer(
            _legacy_segment(negative, stage), angle_limit
        )
        return_rows = [
            sample.get("feedback", sample) for sample in returned
        ]
        return_tail = _tail(return_rows, 0.30)
        return_error = _rms(
            [float(row["control_error_px"]) for row in return_tail]
        )
        return_rate = _rms(
            [float(row["velocity_px_s"]) for row in return_tail]
        )
        return_penalty = 0.20 * return_error + 0.02 * return_rate

    direction_score_gap = abs(
        float(positive_metrics["score"])
        - float(negative_metrics["score"])
    )
    score = (
        0.5
        * (
            float(positive_metrics["score"])
            + float(negative_metrics["score"])
        )
        + return_penalty
        + 0.10 * direction_score_gap
    )
    profile_rows = _profile_samples(samples, sequence, stage)
    phase_durations = {}
    for phase in (
        PROFILE_PREPARE,
        PROFILE_POSITIVE,
        PROFILE_NEGATIVE,
        PROFILE_RETURN,
    ):
        rows = _profile_samples(samples, sequence, stage, phase)
        phase_durations[str(phase)] = max(
            [
                int(row.get("feedback", row).get("phase_elapsed_ms", 0))
                for row in rows
            ]
            or [0]
        )
    return {
        "score": round(score, 6),
        "samples": len(profile_rows),
        "positive": positive_metrics,
        "negative": negative_metrics,
        "return_error": round(return_error, 6),
        "return_rate_rms": round(return_rate, 6),
        "direction_score_gap": round(direction_score_gap, 6),
        "phase_durations_ms": phase_durations,
    }


class ProfileEarlyStop:
    """Cheap streaming rejection of candidates that cannot recover."""

    def __init__(
        self,
        stage,
        sequence,
        minimum_observation_ms=500,
        divergence_ratio=1.60,
        invalid_limit=12,
    ):
        self.stage = str(stage)
        self.sequence = int(sequence)
        self.minimum_observation_ms = int(minimum_observation_ms)
        self.divergence_ratio = float(divergence_ratio)
        self.invalid_limit = int(invalid_limit)
        self.initial_errors = {}
        self.invalid_run = 0

    def observe(self, sample):
        feedback = sample.get("feedback", sample)
        if (
            feedback.get("feedback_version") != 3
            or int(feedback.get("tuning_sequence", -1)) != self.sequence
        ):
            return None
        phase = int(feedback.get("tuning_phase", -1))
        if phase == PROFILE_ABORTED:
            return "mcu_aborted"
        if phase == PROFILE_DONE:
            return None
        if int(feedback.get("protection_state", 0)) != 0:
            return "controller_protection"
        if int(feedback.get("position_valid", 0)) != 1:
            self.invalid_run += 1
            if self.invalid_run >= self.invalid_limit:
                return "position_feedback_lost"
        else:
            self.invalid_run = 0
        if self.stage == "outer" and int(
            feedback.get("vision_age_ms", 999999)
        ) > 200:
            return "vision_feedback_lost"
        if phase not in (PROFILE_POSITIVE, PROFILE_NEGATIVE):
            return None

        error_name = (
            "angle_error_deg"
            if self.stage == "inner"
            else "control_error_px"
        )
        error = abs(float(feedback.get(error_name, 0.0)))
        self.initial_errors.setdefault(phase, max(error, 1e-6))
        elapsed_ms = int(feedback.get("phase_elapsed_ms", 0))
        if (
            elapsed_ms >= self.minimum_observation_ms
            and error > self.initial_errors[phase] * self.divergence_ratio
        ):
            return "response_diverging"
        return None


SEARCH_FACTORS = {
    # Existing hardware logs moved the useful inner-Kp region from the source
    # default 2.5 toward roughly 9..14.  Keep this initial window deliberately
    # wide so one fast study can reuse that evidence instead of repeating six
    # coordinate-search sessions.
    "inner_kp": (0.50, 8.00),
    "inner_kd": (0.0, 0.0),
    "speed_limit": (0.70, 1.50),
    "slew": (0.25, 2.00),
    "outer_kp": (0.45, 2.20),
    "outer_ki": (0.0, 0.0),
    "outer_kd": (0.35, 2.80),
    "angle_limit": (0.65, 1.00),
}


def search_bounds(initial, names):
    bounds = {}
    zero_upper = {
        "inner_kd": 0.50,
        "outer_ki": 0.03,
        "outer_kd": 0.02,
    }
    for name in names:
        value = float(initial[name])
        minimum, maximum = PARAMETER_BOUNDS[name]
        lower_factor, upper_factor = SEARCH_FACTORS[name]
        if abs(value) < 1e-12:
            low, high = 0.0, zero_upper.get(name, maximum)
        else:
            low, high = value * lower_factor, value * upper_factor
        low = max(minimum, min(maximum, low))
        high = max(low, max(minimum, min(maximum, high)))
        bounds[name] = (float(low), float(high))
    return bounds


def optuna_search(
    initial,
    names,
    evaluator,
    trials=16,
    seed=20260801,
    seed_configs=None,
):
    """Joint multivariate TPE search with a reproducible Sobol warm start."""
    try:
        import optuna
    except ImportError as exc:
        raise RuntimeError(
            "Optuna is required; install Vision/windows/requirements.txt"
        ) from exc
    try:
        from scipy.stats import qmc
    except ImportError as exc:
        raise RuntimeError("SciPy is required for Sobol warm-start points") from exc

    names = tuple(names)
    total_trials = max(2, int(trials))
    bounds = search_bounds(initial, names)
    seed_configs = [dict(config) for config in (seed_configs or [])]
    for config in seed_configs:
        for name in names:
            if name not in config:
                continue
            minimum, maximum = PARAMETER_BOUNDS[name]
            value = max(minimum, min(maximum, float(config[name])))
            low, high = bounds[name]
            bounds[name] = (
                max(minimum, min(low, value * 0.90)),
                min(maximum, max(high, value * 1.10)),
            )
    startup = min(6, max(3, total_trials // 3))
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", optuna.exceptions.ExperimentalWarning)
        sampler = optuna.samplers.TPESampler(
            seed=int(seed),
            n_startup_trials=startup,
            multivariate=True,
        )
    study = optuna.create_study(direction="minimize", sampler=sampler)
    queued = [{name: float(initial[name]) for name in names}]
    for config in seed_configs:
        candidate = {name: float(config[name]) for name in names}
        if candidate not in queued:
            queued.append(candidate)
        if len(queued) >= startup:
            break
    for candidate in queued:
        study.enqueue_trial(candidate)
    sobol_count = startup - len(queued)
    if sobol_count > 0:
        engine = qmc.Sobol(d=len(names), scramble=True, seed=int(seed))
        exponent = int(math.ceil(math.log(max(1, sobol_count), 2)))
        points = engine.random_base2(exponent)[:sobol_count]
        for point in points:
            study.enqueue_trial(
                {
                    name: bounds[name][0]
                    + float(point[index])
                    * (bounds[name][1] - bounds[name][0])
                    for index, name in enumerate(names)
                }
            )

    metrics_by_number = {}

    def objective(trial):
        config = dict(initial)
        for name in names:
            low, high = bounds[name]
            if high <= low:
                value = low
            else:
                value = trial.suggest_float(name, low, high)
            config[name] = float(value)
        try:
            metrics = evaluator(config, trial)
        except ExperimentPruned as exc:
            trial.set_user_attr("pruned_reason", str(exc))
            raise optuna.TrialPruned(str(exc))
        metrics_by_number[trial.number] = metrics
        trial.set_user_attr("metrics", metrics)
        return float(metrics["score"])

    study.optimize(objective, n_trials=total_trials)
    completed = [
        trial
        for trial in study.trials
        if trial.state == optuna.trial.TrialState.COMPLETE
    ]
    if not completed:
        raise RuntimeError("all automatic tuning candidates were pruned")
    best = min(completed, key=lambda trial: float(trial.value))
    best_config = dict(initial)
    best_config.update({name: float(value) for name, value in best.params.items()})
    serialized = []
    for trial in study.trials:
        serialized.append(
            {
                "number": trial.number,
                "state": trial.state.name,
                "params": dict(trial.params),
                "score": (
                    None if trial.value is None else float(trial.value)
                ),
                "metrics": metrics_by_number.get(trial.number),
                "pruned_reason": trial.user_attrs.get("pruned_reason"),
            }
        )
    return {
        "best_config": best_config,
        "best_score": float(best.value),
        "bounds": bounds,
        "seed_configs": seed_configs,
        "sampler": "optuna.multivariate_tpe+sobol",
        "trials": serialized,
    }


class ExperimentPruned(RuntimeError):
    pass
