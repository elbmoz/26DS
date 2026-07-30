"""Summarize a MaixCAM dynamic tracking CSV without third-party packages."""

import argparse
import csv
import json
import math
import shutil
import subprocess
from pathlib import Path


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * fraction))
    return ordered[index]


def longest_streak(rows, predicate):
    longest = 0
    current = 0
    for row in rows:
        if predicate(row):
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def ratio(rows, field):
    if not rows:
        return 0.0
    return sum(_boolean(row.get(field)) for row in rows) / len(rows)


def _boolean(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "y")


def _integer(value):
    if value in ("", None):
        return 0
    try:
        return int(value)
    except (TypeError, ValueError):
        return 1 if _boolean(value) else 0


def _float(value, default=0.0):
    if value in ("", None):
        return float(default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def speed_group(row):
    if not _boolean(row.get("valid")):
        return None
    speed = abs(float(row["velocity_px_s"] or 0.0))
    if speed < 80:
        return "slow(<80px/s)"
    if speed < 200:
        return "medium(80-200px/s)"
    return "fast(>=200px/s)"


def probe_video(video_path):
    if video_path is None or not Path(video_path).is_file():
        return None
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        return None
    command = [
        ffprobe,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_name,width,height,avg_frame_rate,nb_frames",
        "-show_entries",
        "format=duration,size",
        "-of",
        "json",
        str(video_path),
    ]
    result = subprocess.run(
        command,
        check=True,
        capture_output=True,
        text=True,
    )
    data = json.loads(result.stdout)
    stream = data["streams"][0]
    media_format = data["format"]
    numerator, denominator = stream["avg_frame_rate"].split("/", 1)
    fps = float(numerator) / max(1.0, float(denominator))
    return {
        "codec": stream.get("codec_name", "unknown"),
        "width": int(stream["width"]),
        "height": int(stream["height"]),
        "fps": fps,
        "frames": int(stream.get("nb_frames") or 0),
        "duration": float(media_format["duration"]),
        "size": int(media_format["size"]),
    }


def align_stream_rows(rows, log_path):
    """Use the frame map written by the Windows receiver when available."""
    frame_map_path = Path(log_path).with_name("video_frames.csv")
    if not frame_map_path.is_file() or "device_ms" not in rows[0]:
        return rows, None
    with frame_map_path.open(newline="", encoding="utf-8") as source:
        frame_rows = list(csv.DictReader(source))
    tracking_ids = [
        int(row["tracking_frame_id"])
        for row in frame_rows
        if row.get("tracking_frame_id") not in ("", None)
    ]
    if len(tracking_ids) < 2:
        return rows, None
    first_id = min(tracking_ids)
    last_id = max(tracking_ids)
    aligned = [
        row
        for row in rows
        if first_id <= int(row["frame_id"]) <= last_id
    ]
    if len(aligned) < 2:
        return rows, None
    video_latency_ms = [
        float(row["video_pipeline_latency_ms"])
        for row in frame_rows
        if row.get("video_pipeline_latency_ms") not in ("", None)
    ]
    sync_delta_ms = [
        abs(float(row["telemetry_match_delta_ms"]))
        for row in frame_rows
        if row.get("telemetry_match_delta_ms") not in ("", None)
    ]
    dropped_frames = sum(
        _integer(row.get("dropped_frames")) for row in frame_rows
    )
    return aligned, {
        "preview_frames": len(frame_rows),
        "first_tracking_frame": first_id,
        "last_tracking_frame": last_id,
        "video_latency_ms": video_latency_ms,
        "sync_delta_ms": sync_delta_ms,
        "dropped_frames": dropped_frames,
    }


def marked_stage_metrics(rows, log_path):
    events_path = Path(log_path).with_name("events.jsonl")
    if not events_path.is_file() or not rows[0].get("host_monotonic_ns"):
        return []
    markers = []
    with events_path.open(encoding="utf-8") as source:
        for line in source:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("type") != "experiment_marker":
                continue
            label = str(
                (event.get("details") or {}).get("label", "")
            ).strip()
            if not label:
                continue
            markers.append(
                {
                    "label": label,
                    "host_monotonic_ns": int(
                        event.get("host_monotonic_ns") or 0
                    ),
                }
            )
    markers.sort(key=lambda item: item["host_monotonic_ns"])
    if not markers:
        return []

    row_times = [int(row["host_monotonic_ns"]) for row in rows]
    stages = []
    for index, marker in enumerate(markers):
        start_ns = marker["host_monotonic_ns"]
        end_ns = (
            markers[index + 1]["host_monotonic_ns"]
            if index + 1 < len(markers)
            else row_times[-1] + 1
        )
        subset = [
            row
            for row, row_time in zip(rows, row_times)
            if start_ns <= row_time < end_ns
        ]
        if not subset:
            continue
        subset_times = [
            int(row["host_monotonic_ns"]) for row in subset
        ]
        duration_s = (
            max(subset_times) - min(subset_times)
        ) / 1_000_000_000.0
        stages.append(
            {
                "label": marker["label"],
                "frames": len(subset),
                "duration_s": max(0.0, duration_s),
                "measured_ratio": ratio(subset, "measured"),
                "valid_ratio": ratio(subset, "valid"),
                "longest_lost_frames": longest_streak(
                    subset,
                    lambda row: not _boolean(row.get("valid")),
                ),
                "detect_ms_p95": percentile(
                    [
                        float(row.get("detect_ms") or 0.0)
                        for row in subset
                    ],
                    0.95,
                ),
                "max_speed_px_s": max(
                    abs(float(row.get("velocity_px_s") or 0.0))
                    for row in subset
                ),
            }
        )
    return stages


def analyze(log_path, video_path=None, return_metrics=False):
    with Path(log_path).open(newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    if not rows:
        raise ValueError("tracking log has no data rows")

    rows, frame_map = align_stream_rows(rows, log_path)
    time_field = "elapsed_ms" if "elapsed_ms" in rows[0] else "device_ms"
    elapsed_ms = int(rows[-1][time_field]) - int(rows[0][time_field])
    seconds = max(0.001, elapsed_ms / 1000.0)
    if video_path is None:
        candidate = Path(log_path).with_name("video.mp4")
        video_path = candidate if candidate.is_file() else None
    media = probe_video(video_path)
    detect_ms = [float(row["detect_ms"]) for row in rows]
    loop_dt_ms = [
        float(row["loop_dt_ms"])
        for row in rows[1:]
        if float(row["loop_dt_ms"]) > 0
    ]
    video_frame_ids = [
        int(row["video_frame_id"])
        for row in rows
        if row.get("video_frame_id") not in ("", None)
    ]
    host_receive_gaps_ms = []
    if rows[0].get("host_monotonic_ns"):
        host_times = [int(row["host_monotonic_ns"]) for row in rows]
        host_receive_gaps_ms = [
            (current - previous) / 1_000_000.0
            for previous, current in zip(host_times, host_times[1:])
            if current > previous
        ]

    groups = {}
    untracked_rows = []
    for row in rows:
        group = speed_group(row)
        if group is None:
            untracked_rows.append(row)
        else:
            groups.setdefault(group, []).append(row)
    stages = marked_stage_metrics(rows, log_path)
    rejection_fields = (
        "position_rejects",
        "lateral_rejects",
        "fixture_rejects",
        "quality_rejects",
        "jump_rejects",
    )
    rejection_counts = {}
    for field in rejection_fields:
        if field not in rows[0]:
            continue
        values = [_integer(row.get(field)) for row in rows]
        rejection_counts[field] = {
            "candidates": sum(values),
            "frames": sum(value > 0 for value in values),
        }

    algorithm = str(rows[0].get("algorithm") or "").strip()
    pipe_metrics = None
    if "pipe_valid" in rows[0] and algorithm != "ai":
        pose_rows = [
            row
            for row in rows
            if all(
                row.get(field) not in ("", None)
                for field in ("axis_x0", "axis_y0", "axis_x1", "axis_y1")
            )
        ]
        centers_x = []
        centers_y = []
        angles_deg = []
        for row in pose_rows:
            x0 = _float(row.get("axis_x0"))
            y0 = _float(row.get("axis_y0"))
            x1 = _float(row.get("axis_x1"))
            y1 = _float(row.get("axis_y1"))
            centers_x.append(0.5 * (x0 + x1))
            centers_y.append(0.5 * (y0 + y1))
            angles_deg.append(math.degrees(math.atan2(y1 - y0, x1 - x0)))
        pipe_metrics = {
            "valid_ratio": ratio(rows, "pipe_valid"),
            "measured_ratio": ratio(rows, "pipe_measured"),
            "max_age_frames": max(
                _integer(row.get("pipe_age_frames")) for row in rows
            ),
            "center_x_span_px": (
                max(centers_x) - min(centers_x) if centers_x else 0.0
            ),
            "center_y_span_px": (
                max(centers_y) - min(centers_y) if centers_y else 0.0
            ),
            "angle_min_deg": min(angles_deg) if angles_deg else 0.0,
            "angle_max_deg": max(angles_deg) if angles_deg else 0.0,
        }

    rate_label = (
        "logged telemetry rate"
        if time_field == "device_ms"
        else "effective FPS"
    )
    lines = [
        "frames: {}".format(len(rows)),
        "duration: {:.2f} s".format(seconds),
        "{}: {:.2f}".format(rate_label, (len(rows) - 1) / seconds),
        "measured hit ratio: {:.2%}".format(ratio(rows, "measured")),
        "valid output ratio: {:.2%}".format(ratio(rows, "valid")),
        "prediction coast ratio: {:.2%}".format(ratio(rows, "coasting")),
        "longest lost streak: {} frames".format(
            longest_streak(rows, lambda row: not _boolean(row["valid"]))
        ),
        "longest coast streak: {} frames".format(
            longest_streak(rows, lambda row: _boolean(row["coasting"]))
        ),
        "detect time p50/p95/max: {:.1f}/{:.1f}/{:.1f} ms".format(
            percentile(detect_ms, 0.50),
            percentile(detect_ms, 0.95),
            max(detect_ms),
        ),
        "loop dt p50/p95/max: {:.1f}/{:.1f}/{:.1f} ms".format(
            percentile(loop_dt_ms, 0.50),
            percentile(loop_dt_ms, 0.95),
            max(loop_dt_ms) if loop_dt_ms else 0.0,
        ),
        (
            "evaluation note: whole-session measured/valid ratios are "
            "operational availability, not labeled accuracy; use marked "
            "experiment stages for comparisons"
        ),
    ]
    if algorithm:
        lines.insert(0, "algorithm: {}".format(algorithm))
    if algorithm == "ai":
        lines.append(
            "coordinate reference: fixed installation calibration; "
            "no image-based pipe pose was evaluated"
        )
    if frame_map is not None:
        lines.insert(
            0,
            "analysis window: {} decoded video frames, tracking {}..{}".format(
                frame_map["preview_frames"],
                frame_map["first_tracking_frame"],
                frame_map["last_tracking_frame"],
            ),
        )
        video_latency_ms = frame_map.get("video_latency_ms", [])
        if video_latency_ms:
            lines.append(
                "video pipeline latency p50/p95/max: "
                "{:.1f}/{:.1f}/{:.1f} ms".format(
                    percentile(video_latency_ms, 0.50),
                    percentile(video_latency_ms, 0.95),
                    max(video_latency_ms),
                )
            )
        sync_delta_ms = frame_map.get("sync_delta_ms", [])
        if sync_delta_ms:
            lines.append(
                "video/telemetry alignment error p50/p95/max: "
                "{:.1f}/{:.1f}/{:.1f} ms".format(
                    percentile(sync_delta_ms, 0.50),
                    percentile(sync_delta_ms, 0.95),
                    max(sync_delta_ms),
                )
            )
        lines.append(
            "preview frames intentionally dropped: {}".format(
                frame_map.get("dropped_frames", 0)
            )
        )
    detector_fps = []
    if time_field == "device_ms":
        detector_fps = [
            float(row["fps"])
            for row in rows
            if row.get("fps") not in ("", None)
        ]
        if detector_fps:
            lines.insert(
                4,
                "reported detector FPS mean/p05/p95: "
                "{:.2f}/{:.2f}/{:.2f}".format(
                    sum(detector_fps) / len(detector_fps),
                    percentile(detector_fps, 0.05),
                    percentile(detector_fps, 0.95),
                ),
            )
    if video_frame_ids:
        nominal_span = max(video_frame_ids) - min(video_frame_ids)
        lines.insert(
            3,
            "nominal video timeline FPS: {:.2f}".format(
                nominal_span / seconds
            ),
        )
    if media is not None:
        lines.insert(
            3,
            "video: {} {}x{}, {:.2f} FPS, {} frames, {:.2f} s".format(
                media["codec"],
                media["width"],
                media["height"],
                media["fps"],
                media["frames"],
                media["duration"],
            ),
        )
        lines.insert(
            4,
            "log/video duration delta: {:.3f} s".format(
                abs(seconds - media["duration"])
            ),
        )
    if host_receive_gaps_ms:
        lines.append(
            "telemetry receive gap p50/p95/max: "
            "{:.1f}/{:.1f}/{:.1f} ms".format(
                percentile(host_receive_gaps_ms, 0.50),
                percentile(host_receive_gaps_ms, 0.95),
                max(host_receive_gaps_ms),
            )
        )
    if rejection_counts:
        lines.append(
            "candidate rejects position/lateral/fixture/quality/jump: "
            "{}/{}/{}/{}/{} candidates".format(
                *[
                    rejection_counts.get(field, {}).get("candidates", 0)
                    for field in rejection_fields
                ]
            )
        )
    if pipe_metrics is not None:
        lines.append(
            "pipe pose valid/measured: {:.2%}/{:.2%}, max age {} frames".format(
                pipe_metrics["valid_ratio"],
                pipe_metrics["measured_ratio"],
                pipe_metrics["max_age_frames"],
            )
        )
        lines.append(
            "pipe motion center span x/y: {:.1f}/{:.1f} px, "
            "angle range: {:.2f}..{:.2f} deg".format(
                pipe_metrics["center_x_span_px"],
                pipe_metrics["center_y_span_px"],
                pipe_metrics["angle_min_deg"],
                pipe_metrics["angle_max_deg"],
            )
        )
    if untracked_rows:
        lines.append(
            "untracked (speed unknown): {} frames ({:.2%})".format(
                len(untracked_rows),
                len(untracked_rows) / len(rows),
            )
        )
    for name in ("slow(<80px/s)", "medium(80-200px/s)", "fast(>=200px/s)"):
        subset = groups.get(name, [])
        if subset:
            lines.append(
                "{}: {} frames, measured {:.2%}, valid {:.2%}".format(
                    name,
                    len(subset),
                    ratio(subset, "measured"),
                    ratio(subset, "valid"),
                )
            )
    if stages:
        lines.append("marked experiment stages:")
        for stage in stages:
            lines.append(
                "  {}: {} frames/{:.2f}s, measured {:.2%}, "
                "valid {:.2%}, longest_lost {} frames, "
                "detect_p95 {:.1f}ms, max_speed {:.0f}px/s".format(
                    stage["label"],
                    stage["frames"],
                    stage["duration_s"],
                    stage["measured_ratio"],
                    stage["valid_ratio"],
                    stage["longest_lost_frames"],
                    stage["detect_ms_p95"],
                    stage["max_speed_px_s"],
                )
            )
    report = "\n".join(lines)
    if not return_metrics:
        return report

    metrics = {
        "algorithm": algorithm or None,
        "frames": len(rows),
        "duration_s": seconds,
        "rate_label": rate_label,
        "logged_rate_hz": (len(rows) - 1) / seconds,
        "reported_detector_fps": (
            {
                "mean": sum(detector_fps) / len(detector_fps),
                "p05": percentile(detector_fps, 0.05),
                "p95": percentile(detector_fps, 0.95),
            }
            if detector_fps
            else None
        ),
        "measured_ratio": ratio(rows, "measured"),
        "valid_ratio": ratio(rows, "valid"),
        "coasting_ratio": ratio(rows, "coasting"),
        "evaluation": {
            "whole_session_metric": "operational_availability",
            "labeled_accuracy_available": bool(stages),
            "accuracy_requires_marked_stage": True,
        },
        "untracked_frames": len(untracked_rows),
        "untracked_ratio": len(untracked_rows) / len(rows),
        "longest_lost_frames": longest_streak(
            rows, lambda row: not _boolean(row["valid"])
        ),
        "longest_coast_frames": longest_streak(
            rows, lambda row: _boolean(row["coasting"])
        ),
        "detect_ms": {
            "p50": percentile(detect_ms, 0.50),
            "p95": percentile(detect_ms, 0.95),
            "max": max(detect_ms),
        },
        "loop_dt_ms": {
            "p50": percentile(loop_dt_ms, 0.50),
            "p95": percentile(loop_dt_ms, 0.95),
            "max": max(loop_dt_ms) if loop_dt_ms else 0.0,
        },
    }
    if frame_map is not None:
        latency = frame_map.get("video_latency_ms", [])
        sync = frame_map.get("sync_delta_ms", [])
        metrics["preview_frames"] = frame_map["preview_frames"]
        metrics["dropped_preview_frames"] = frame_map.get(
            "dropped_frames", 0
        )
        if latency:
            metrics["video_pipeline_latency_ms"] = {
                "p50": percentile(latency, 0.50),
                "p95": percentile(latency, 0.95),
                "max": max(latency),
            }
        if sync:
            metrics["video_telemetry_alignment_error_ms"] = {
                "p50": percentile(sync, 0.50),
                "p95": percentile(sync, 0.95),
                "max": max(sync),
            }
    if media is not None:
        metrics["video"] = media
        metrics["log_video_duration_delta_s"] = abs(
            seconds - media["duration"]
        )
    if host_receive_gaps_ms:
        metrics["telemetry_receive_gap_ms"] = {
            "p50": percentile(host_receive_gaps_ms, 0.50),
            "p95": percentile(host_receive_gaps_ms, 0.95),
            "max": max(host_receive_gaps_ms),
        }
    metrics["speed_groups"] = {
        name: {
            "frames": len(subset),
            "measured_ratio": ratio(subset, "measured"),
            "valid_ratio": ratio(subset, "valid"),
        }
        for name, subset in groups.items()
    }
    metrics["marked_stages"] = stages
    metrics["candidate_rejections"] = rejection_counts
    if pipe_metrics is not None:
        metrics["pipe_pose"] = pipe_metrics
    return report, metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("tracking_csv", type=Path)
    parser.add_argument("--video", type=Path)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--text-out", type=Path)
    args = parser.parse_args()
    report, metrics = analyze(
        args.tracking_csv, args.video, return_metrics=True
    )
    print(report)
    if args.text_out is not None:
        args.text_out.write_text(report + "\n", encoding="utf-8")
    if args.json_out is not None:
        args.json_out.write_text(
            json.dumps(
                {
                    "schema": 1,
                    "tracking_csv": str(args.tracking_csv),
                    "video": (
                        None if args.video is None else str(args.video)
                    ),
                    "metrics": metrics,
                    "report": report,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )


if __name__ == "__main__":
    main()
