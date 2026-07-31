"""Build a reproducible motor-position / Q9 X-angle calibration report.

The Windows receiver logs the latest Q9 payload on every tracking packet, so a
single 5 Hz Q9 frame normally appears many times in ``telemetry.jsonl``.  This
tool deduplicates those payloads, keeps invalid and transient samples visible
in the exported CSV, and fits the settled relationship with a small robust
linear regression.  It intentionally uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


RESIDUAL_LIMIT_DEG = 0.75


def _linear_fit(rows):
    count = len(rows)
    if count < 2:
        raise ValueError("at least two samples are required")
    mean_x = sum(row["motor_position"] for row in rows) / count
    mean_y = sum(row["angle_x_deg"] for row in rows) / count
    covariance = sum(
        (row["motor_position"] - mean_x) * (row["angle_x_deg"] - mean_y)
        for row in rows
    )
    variance = sum(
        (row["motor_position"] - mean_x) ** 2 for row in rows
    )
    if variance == 0:
        raise ValueError("motor position does not vary")
    slope = covariance / variance
    intercept = mean_y - slope * mean_x
    residuals = [
        row["angle_x_deg"] - (slope * row["motor_position"] + intercept)
        for row in rows
    ]
    squared_error = sum(value * value for value in residuals)
    absolute_error = sum(abs(value) for value in residuals)
    total_y_error = sum(
        (row["angle_x_deg"] - mean_y) ** 2 for row in rows
    )
    return {
        "slope_deg_per_count": slope,
        "intercept_deg": intercept,
        "zero_angle_position": -intercept / slope,
        "counts_per_degree": abs(1.0 / slope),
        "sample_count": count,
        "rmse_deg": math.sqrt(squared_error / count),
        "mae_deg": absolute_error / count,
        "max_abs_error_deg": max(abs(value) for value in residuals),
        "r_squared": (
            1.0 - squared_error / total_y_error
            if total_y_error > 0
            else 1.0
        ),
    }


def _robust_fit(rows, residual_limit_deg):
    selected = list(rows)
    for _ in range(20):
        fit = _linear_fit(selected)
        next_selected = [
            row
            for row in rows
            if abs(
                row["angle_x_deg"]
                - (
                    fit["slope_deg_per_count"] * row["motor_position"]
                    + fit["intercept_deg"]
                )
            )
            <= residual_limit_deg
        ]
        if len(next_selected) == len(selected):
            selected = next_selected
            break
        if len(next_selected) < 2:
            raise ValueError("robust rejection removed too many samples")
        selected = next_selected
    return _linear_fit(selected), selected


def _load_samples(path):
    samples = []
    seen = set()
    parse_errors = 0
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            try:
                packet = json.loads(line)
            except (TypeError, ValueError):
                parse_errors += 1
                continue
            q9 = packet.get("q9")
            if not isinstance(q9, dict):
                continue
            identity = (q9.get("seq"), q9.get("mcu_ms"))
            if identity in seen:
                continue
            seen.add(identity)
            try:
                sample = {
                    "source_line": line_number,
                    "host_epoch_ns": int(
                        packet.get("host_epoch_ns")
                        or packet.get("_host_epoch_ns")
                        or 0
                    ),
                    "seq": int(q9["seq"]),
                    "seq_gap": int(q9.get("seq_gap", 0)),
                    "mcu_ms": int(q9["mcu_ms"]),
                    "motor_position": int(q9["motor_position"]),
                    "angle_x_x10": int(q9["angle_x_x10"]),
                    "angle_x_deg": float(q9["angle_x_x10"]) / 10.0,
                    "angle_y_x10": int(q9["angle_y_x10"]),
                    "angle_y_deg": float(q9["angle_y_x10"]) / 10.0,
                    "angle_z_x10": int(q9["angle_z_x10"]),
                    "angle_z_deg": float(q9["angle_z_x10"]) / 10.0,
                    "imu_valid": int(q9["imu_valid"]),
                    "position_valid": int(q9["position_valid"]),
                    "position_status": int(q9["position_status"]),
                    "position_updates": int(q9["position_updates"]),
                    "move_direction": int(q9["move_direction"]),
                    "move_status": int(q9["move_status"]),
                }
            except (KeyError, TypeError, ValueError):
                parse_errors += 1
                continue
            samples.append(sample)
    return samples, parse_errors


def _interpolate_position(samples, timestamp_ns):
    if timestamp_ns < samples[0]["host_epoch_ns"]:
        return None
    if timestamp_ns > samples[-1]["host_epoch_ns"]:
        return None
    low = 0
    high = len(samples) - 1
    while low + 1 < high:
        middle = (low + high) // 2
        if samples[middle]["host_epoch_ns"] <= timestamp_ns:
            low = middle
        else:
            high = middle
    left = samples[low]
    right = samples[high]
    span = right["host_epoch_ns"] - left["host_epoch_ns"]
    if span <= 0:
        return float(left["motor_position"])
    fraction = (timestamp_ns - left["host_epoch_ns"]) / span
    return left["motor_position"] + fraction * (
        right["motor_position"] - left["motor_position"]
    )


def _lag_scan(valid_samples):
    timed = [
        row for row in valid_samples if row["host_epoch_ns"] > 0
    ]
    if len(timed) < 3:
        return None
    best = None
    for lag_ms in range(-200, 205, 5):
        lag_ns = lag_ms * 1_000_000
        shifted = []
        for row in timed:
            position = _interpolate_position(
                timed, row["host_epoch_ns"] + lag_ns
            )
            if position is None:
                continue
            shifted.append(
                {
                    "motor_position": position,
                    "angle_x_deg": row["angle_x_deg"],
                }
            )
        if len(shifted) < 3:
            continue
        fit = _linear_fit(shifted)
        candidate = {
            "position_time_offset_ms": lag_ms,
            "sample_count": fit["sample_count"],
            "rmse_deg": fit["rmse_deg"],
        }
        if best is None or candidate["rmse_deg"] < best["rmse_deg"]:
            best = candidate
    return best


def _write_samples(path, samples, fit, inlier_ids):
    fieldnames = [
        "source_line",
        "host_epoch_ns",
        "seq",
        "seq_gap",
        "mcu_ms",
        "motor_position",
        "angle_x_x10",
        "angle_x_deg",
        "angle_y_x10",
        "angle_y_deg",
        "angle_z_x10",
        "angle_z_deg",
        "imu_valid",
        "position_valid",
        "position_status",
        "position_updates",
        "move_direction",
        "move_status",
        "fit_eligible",
        "fit_inlier",
        "fit_residual_deg",
    ]
    with path.open("w", encoding="utf-8-sig", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=fieldnames)
        writer.writeheader()
        for row in samples:
            eligible = row["imu_valid"] == 1 and row["position_valid"] == 1
            residual = (
                row["angle_x_deg"]
                - (
                    fit["slope_deg_per_count"] * row["motor_position"]
                    + fit["intercept_deg"]
                )
                if eligible
                else None
            )
            output = dict(row)
            output["fit_eligible"] = int(eligible)
            output["fit_inlier"] = int(id(row) in inlier_ids)
            output["fit_residual_deg"] = (
                "" if residual is None else "{:.6f}".format(residual)
            )
            writer.writerow(output)


def _write_report(path, source_path, summary):
    robust = summary["robust_linear_fit"]
    direction = summary["direction_fits"]
    table = summary["mapping_table"]
    lines = [
        "# 任务 9 电机位置—管道 X 轴角度映射",
        "",
        "数据源：`{}`".format(source_path.name),
        "",
        "## 结论",
        "",
        "可信稳态样本呈线性关系，无需二次曲线或复杂查表：",
        "",
        "```text",
        "X_deg = {slope:.10f} * P + {intercept:.7f}".format(
            slope=robust["slope_deg_per_count"],
            intercept=robust["intercept_deg"],
        ),
        "P = {zero:.2f} - {counts:.2f} * X_deg".format(
            zero=robust["zero_angle_position"],
            counts=robust["counts_per_degree"],
        ),
        "```",
        "",
        "每增加约 `{:.2f}` 个电机位置计数，X 角度下降 1°。".format(
            robust["counts_per_degree"]
        ),
        "",
        "由于任务 9 每次确认都会重新将相对角度归零，跨任务运行应使用锚点形式：",
        "",
        "```text",
        "X_est = X_ref - {scale:.10f} * (P - P_ref)".format(
            scale=abs(robust["slope_deg_per_count"])
        ),
        "```",
        "",
        "本次运行拟合的 `X_ref=0°` 锚点为 `P_ref={:.2f}`；"
        "该绝对零点不能直接沿用到下一次任务运行。".format(
            robust["zero_angle_position"]
        ),
        "",
        "## 拟合质量",
        "",
        "- Q9 去重帧：{}；有效帧：{}；稳态内点：{}；动态离群点：{}。".format(
            summary["counts"]["unique_q9_frames"],
            summary["counts"]["fit_eligible_frames"],
            summary["counts"]["robust_inliers"],
            summary["counts"]["robust_outliers"],
        ),
        "- RMSE：`{:.3f}°`；MAE：`{:.3f}°`；R²：`{:.6f}`。".format(
            robust["rmse_deg"],
            robust["mae_deg"],
            robust["r_squared"],
        ),
        "- 适用范围：电机位置 `{:.0f}..{:.0f}`，X 角度 `{:.1f}..{:.1f}°`。".format(
            summary["validated_range"]["motor_position_min"],
            summary["validated_range"]["motor_position_max"],
            summary["validated_range"]["angle_x_deg_min"],
            summary["validated_range"]["angle_x_deg_max"],
        ),
        "- 最佳时间对齐显示电机位置数据约落后角度 `{:+d} ms`；"
        "这解释了快速移动和换向处的大残差。".format(
            summary["timing"]["best_position_time_offset_ms"]
        ),
        "",
        "## 建议目标值",
        "",
        "| X 角度 | 电机位置 |",
        "|---:|---:|",
    ]
    for item in table:
        lines.append("| {:+.0f}° | {:.0f} |".format(
            item["angle_x_deg"], item["motor_position"]
        ))
    lines.extend(
        [
            "",
            "## 动态样本处理",
            "",
            "位置到角度的映射用于稳态标定。运动中优先使用 IMU X 角度；"
            "若必须使用映射，至少要求 `imu_valid=1`、`position_valid=1`、"
            "`position_status=0`，换向后等待 400–600 ms，并确认连续两帧"
            "`|ΔP| < 50`。",
            "",
            "正反向拟合零点相差约 `{:.1f}` 个计数，等效 `{:.3f}°`，"
            "当前精度下不建议增加方向补偿。".format(
                summary["direction_hysteresis"]["zero_position_difference"],
                summary["direction_hysteresis"]["equivalent_angle_deg"],
            ),
            "",
        "方向 -1：`P0={:.2f}`，RMSE `{:.3f}°`，n={}；"
        "方向 +1：`P0={:.2f}`，RMSE `{:.3f}°`，n={}。".format(
                direction["-1"]["zero_angle_position"],
                direction["-1"]["rmse_deg"],
                direction["-1"]["sample_count"],
                direction["1"]["zero_angle_position"],
                direction["1"]["rmse_deg"],
                direction["1"]["sample_count"],
            ),
            "",
            "## 数据包内容",
            "",
            "- `telemetry.jsonl`：Windows 上位机收到的原始遥测。",
            "- `video.mp4`：与遥测同期录制的 RTSP 画面。",
            "- `video_frames.csv`：录像帧与主机时间轴。",
            "- `q9_motor_angle_samples.csv`：去重后的全部 Q9 帧、拟合资格、"
            "内点标记和残差。",
            "- `q9_motor_angle_fit.json`：机器可读拟合参数和统计量。",
            "- `q9_mapping_video_contact.jpg`：换向和两端姿态关键帧对照。",
            "",
            "复算命令：",
            "",
            "```powershell",
            "python Vision\\tools\\analyze_q9_motor_angle.py `",
            "  Vision\\captures\\stream_sessions\\stream_20260731_151006"
            "\\telemetry.jsonl",
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def analyze(telemetry_path, output_dir):
    samples, parse_errors = _load_samples(telemetry_path)
    valid = [
        row
        for row in samples
        if row["imu_valid"] == 1 and row["position_valid"] == 1
    ]
    if len(valid) < 2:
        raise ValueError("not enough valid Q9 samples")
    raw_fit = _linear_fit(valid)
    robust_fit, inliers = _robust_fit(valid, RESIDUAL_LIMIT_DEG)
    direction_fits = {}
    for direction in (-1, 1):
        directed = [
            row for row in inliers if row["move_direction"] == direction
        ]
        direction_fits[str(direction)] = _linear_fit(directed)
    zero_difference = abs(
        direction_fits["-1"]["zero_angle_position"]
        - direction_fits["1"]["zero_angle_position"]
    )
    lag = _lag_scan(valid)
    if lag is None:
        lag = {
            "position_time_offset_ms": 0,
            "sample_count": 0,
            "rmse_deg": raw_fit["rmse_deg"],
        }
    mapping_table = []
    for angle in (-8, -6, -4, -2, 0, 2, 4, 6):
        position = (
            angle - robust_fit["intercept_deg"]
        ) / robust_fit["slope_deg_per_count"]
        mapping_table.append(
            {"angle_x_deg": angle, "motor_position": position}
        )
    summary = {
        "schema_version": 1,
        "source_telemetry": telemetry_path.name,
        "residual_limit_deg": RESIDUAL_LIMIT_DEG,
        "counts": {
            "unique_q9_frames": len(samples),
            "fit_eligible_frames": len(valid),
            "robust_inliers": len(inliers),
            "robust_outliers": len(valid) - len(inliers),
            "json_or_q9_parse_errors": parse_errors,
        },
        "validated_range": {
            "motor_position_min": min(row["motor_position"] for row in valid),
            "motor_position_max": max(row["motor_position"] for row in valid),
            "angle_x_deg_min": min(row["angle_x_deg"] for row in valid),
            "angle_x_deg_max": max(row["angle_x_deg"] for row in valid),
        },
        "raw_linear_fit": raw_fit,
        "robust_linear_fit": robust_fit,
        "direction_fits": direction_fits,
        "direction_hysteresis": {
            "zero_position_difference": zero_difference,
            "equivalent_angle_deg": (
                zero_difference
                * abs(robust_fit["slope_deg_per_count"])
            ),
        },
        "timing": {
            "best_position_time_offset_ms": lag[
                "position_time_offset_ms"
            ],
            "aligned_raw_rmse_deg": lag["rmse_deg"],
            "interpretation": (
                "positive means comparing current angle with a future "
                "position sample improves alignment"
            ),
        },
        "mapping_table": mapping_table,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    fit_path = output_dir / "q9_motor_angle_fit.json"
    csv_path = output_dir / "q9_motor_angle_samples.csv"
    report_path = output_dir / "Q9_MOTOR_ANGLE_MAPPING.md"
    fit_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_samples(
        csv_path, samples, robust_fit, {id(row) for row in inliers}
    )
    _write_report(report_path, telemetry_path, summary)
    return fit_path, csv_path, report_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="defaults to the telemetry file directory",
    )
    args = parser.parse_args()
    telemetry_path = args.telemetry.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else telemetry_path.parent
    )
    for path in analyze(telemetry_path, output_dir):
        print(path)


if __name__ == "__main__":
    main()
