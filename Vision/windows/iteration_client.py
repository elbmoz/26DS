"""Small CLI for Codex or a teammate to control the running monitor."""

import argparse
import json
import os
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from pid_auto_tuner import (
    INNER_PARAMETER_NAMES,
    OUTER_PARAMETER_NAMES,
    coordinate_search,
    score_inner,
    score_outer,
)


ALIASES = {
    "目标位置": "target_position",
    "位置跟随": "position_alpha",
    "速度修正": "velocity_beta",
    "横向平滑": "lateral_alpha",
    "中心线范围": "max_axis_distance_px",
    "中心线下方范围": "max_below_axis_distance_px",
    "最大跳动": "max_frame_jump_px",
    "重找端点容差": "acquire_position_margin",
    "端点容差": "track_position_margin",
    "首次找球端部禁区": "acquire_endpoint_inset",
    "跟踪端部禁区": "track_endpoint_inset",
    "重找可信度": "acquire_min_quality",
    "跟踪可信度": "track_min_quality",
    "续航帧数": "coast_frames",
    "搜索宽度": "local_search_width_px",
    "圆检测严格度": "circle_threshold",
    "钢球最小半径": "circle_min_radius",
    "钢球最大半径": "circle_max_radius",
}

PID_ALIASES = {
    "outer_ki_deg_per_px_s": "outer_ki",
    "outer_angle_limit": "angle_limit",
    "angle_kp_speed_per_deg": "inner_kp",
    "angle_kd_speed_per_deg_s": "inner_kd",
    "motor_speed_limit": "speed_limit",
    "motor_slew_per_update": "slew",
    "motor_speed_deadband": "deadband",
    "motor_min_speed": "min_speed",
}


def _request(base_url, path, body=None):
    payload = None
    method = "GET"
    headers = {}
    if body is not None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        method = "POST"
        headers["Content-Type"] = "application/json; charset=utf-8"
    request = Request(
        base_url.rstrip("/") + path,
        data=payload,
        headers=headers,
        method=method,
    )
    try:
        with urlopen(request, timeout=3.0) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError("HTTP {}: {}".format(exc.code, detail))
    except URLError as exc:
        raise RuntimeError(
            "监控程序未运行或接口不可达: {}".format(exc.reason)
        )


def _assignments(values):
    params = {}
    for assignment in values:
        if "=" not in assignment:
            raise ValueError("参数必须写成 名称=数值")
        name, raw_value = assignment.split("=", 1)
        name = ALIASES.get(name.strip(), name.strip())
        value = float(raw_value.strip())
        if value.is_integer():
            value = int(value)
        params[name] = value
    return params


def _pid_assignments(values):
    params = _assignments(values)
    return {PID_ALIASES.get(name, name): value for name, value in params.items()}


def _wait_command(base_url, accepted, timeout=7.0):
    command_id = accepted.get("command_id")
    if not command_id:
        raise RuntimeError("iteration API did not return a command id")
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        status = _request(base_url, "/status")
        command = status.get("last_command") or {}
        if command.get("id") == command_id:
            state = command.get("state")
            if state == "applied":
                return command.get("ack", command)
            if state in ("rejected", "ack_timeout"):
                raise RuntimeError(
                    "command {}: {}".format(state, command)
                )
        time.sleep(0.10)
    raise RuntimeError("command {} timed out".format(command_id))


def _submit(base_url, path, body=None, timeout=7.0):
    accepted = _request(base_url, path, {} if body is None else body)
    return _wait_command(base_url, accepted, timeout=timeout)


def _parse_pid_test(values):
    result = {}
    for assignment in values:
        if "=" not in assignment:
            raise ValueError("test arguments must use name=value")
        name, value = assignment.split("=", 1)
        name = name.strip()
        value = value.strip()
        if name == "mode":
            result["mode"] = {
                "inner_step": "inner",
                "outer_step": "outer",
            }.get(value, value)
        elif name in ("target", "target_angle", "target_position"):
            result["target"] = float(value)
        elif name == "duration":
            result["duration_ms"] = int(round(float(value) * 1000.0))
        elif name == "duration_ms":
            result["duration_ms"] = int(value)
        else:
            raise ValueError("unknown PID test argument {}".format(name))
    if set(result) != {"mode", "target", "duration_ms"}:
        raise ValueError("PID test requires mode, target and duration")
    return result


def _run_pid_test(base_url, mode, target, duration_s):
    stream_request = Request(
        base_url.rstrip("/") + "/telemetry",
        headers={"Accept": "text/event-stream"},
        method="GET",
    )
    samples = []
    with urlopen(
        stream_request, timeout=max(8.0, float(duration_s) + 6.0)
    ) as response:
        ack = _submit(
            base_url,
            "/pid/test",
            {
                "mode": mode,
                "target": float(target),
                "duration_ms": int(round(float(duration_s) * 1000.0)),
            },
        )
        deadline = time.monotonic() + float(duration_s) + 0.6
        while time.monotonic() < deadline:
            line = response.readline()
            if not line:
                break
            if not line.startswith(b"data: "):
                continue
            try:
                sample = json.loads(line[6:].decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if sample.get("feedback"):
                samples.append(sample)
    return ack, samples


def _run_pid_auto(args):
    initial_ack = _submit(args.url, "/pid")
    initial = dict(initial_ack.get("config", {}))
    names = (
        INNER_PARAMETER_NAMES
        if args.stage == "inner"
        else OUTER_PARAMETER_NAMES
    )
    missing = [name for name in names if name not in initial]
    if missing:
        raise RuntimeError("STM32 PID ACK is missing {}".format(missing))

    targets = (float(args.target), -float(args.target))
    recenter_duration = max(5.0, min(8.0, 2.0 * float(args.duration)))
    outer_center_duration = max(
        8.0, min(12.0, 2.5 * float(args.duration))
    )

    def recenter_rod():
        last_angle = None
        for _attempt in range(2):
            _ack, samples = _run_pid_test(
                args.url, "inner", 0.0, recenter_duration
            )
            angles = []
            for sample in samples:
                feedback = sample.get("feedback") or {}
                if (
                    feedback.get("feedback_version") == 2
                    and feedback.get("tuning_mode") == 1
                    and feedback.get("position_valid") == 1
                ):
                    angles.append(float(feedback["actual_rod_angle_deg"]))
            if angles:
                tail = angles[-min(5, len(angles)) :]
                last_angle = sum(tail) / float(len(tail))
                if abs(last_angle) <= 0.35:
                    return round(last_angle, 5)
        raise RuntimeError(
            "inner recenter did not reach +/-0.35 deg (last={})".format(
                last_angle
            )
        )

    def center_ball():
        last_error_rms = None
        last_velocity_rms = None
        for _attempt in range(2):
            _ack, samples = _run_pid_test(
                args.url, "outer", 0.0, outer_center_duration
            )
            rows = []
            for sample in samples:
                feedback = sample.get("feedback") or {}
                tracking = sample.get("tracking")
                if (
                    feedback.get("feedback_version") == 2
                    and feedback.get("tuning_mode") == 2
                    and feedback.get("position_valid") == 1
                    and feedback.get("vision_age_ms", 999999) <= 200
                    and not (
                        isinstance(tracking, dict)
                        and tracking.get("valid") is False
                    )
                ):
                    rows.append(feedback)
            if len(rows) < 10:
                continue
            tail = rows[-max(10, int(round(0.25 * len(rows)))) :]
            errors = [float(row["control_error_px"]) for row in tail]
            velocities = [float(row["velocity_px_s"]) for row in tail]
            last_error_rms = (
                sum(value * value for value in errors) / len(errors)
            ) ** 0.5
            last_velocity_rms = (
                sum(value * value for value in velocities) / len(velocities)
            ) ** 0.5
            if last_error_rms <= 25.0 and last_velocity_rms <= 80.0:
                return {
                    "start_position_error_rms_px": round(last_error_rms, 5),
                    "start_velocity_rms_px_s": round(last_velocity_rms, 5),
                    "start_angle_deg": round(
                        sum(
                            float(row["actual_rod_angle_deg"])
                            for row in tail
                        )
                        / len(tail),
                        5,
                    ),
                }
        raise RuntimeError(
            "outer centering did not settle "
            "(error_rms={}, velocity_rms={}); check that the ball is present".format(
                last_error_rms, last_velocity_rms
            )
        )

    def prepare_start():
        if args.stage == "outer":
            return center_ball()
        return {"start_angle_deg": recenter_rod()}

    def evaluator(config):
        _submit(args.url, "/pid/set", {"params": config})
        runs = []
        for target in targets:
            start_state = prepare_start()
            _ack, samples = _run_pid_test(
                args.url, args.stage, target, args.duration
            )
            if args.stage == "inner":
                metrics = score_inner(samples, config["speed_limit"])
            else:
                metrics = score_outer(samples, config["angle_limit"])
            metrics["target"] = target
            metrics.update(start_state)
            runs.append(metrics)
        return {
            "score": sum(run["score"] for run in runs) / len(runs),
            "runs": runs,
        }

    try:
        result = coordinate_search(
            initial,
            names,
            evaluator,
            rounds=args.rounds,
        )
        final_ack = _submit(
            args.url, "/pid/set", {"params": result["best_config"]}
        )
        final_recenter_angle = recenter_rod()
        result.update(
            {
                "stage": args.stage,
                "initial_config": initial,
                "final_ack": final_ack,
                "duration_s": args.duration,
                "recenter_duration_s": recenter_duration,
                "outer_center_duration_s": (
                    outer_center_duration if args.stage == "outer" else None
                ),
                "final_recenter_angle_deg": final_recenter_angle,
                "targets": list(targets),
                "completed_epoch_ns": time.time_ns(),
            }
        )
    finally:
        try:
            _submit(args.url, "/pid/stop")
        except Exception:
            pass

    output = (
        Path(__file__).resolve().parents[1]
        / "runtime"
        / "pid_auto_{}_{}.json".format(
            args.stage, time.strftime("%Y%m%d_%H%M%S")
        )
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    result["report"] = str(output)
    return result


def build_parser():
    default_status_file = (
        Path(__file__).resolve().parents[1]
        / "runtime"
        / "live_status.json"
    )
    parser = argparse.ArgumentParser(
        description="连接正在运行的滚球视觉监控实验"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get(
            "VISION_ITERATION_URL", "http://127.0.0.1:8765"
        ),
    )
    parser.add_argument(
        "--status-file",
        type=Path,
        default=default_status_file,
        help="监控结束后读取的最后状态文件",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="读取实时状态")
    subparsers.add_parser("report", help="读取最近一次实验的自动分析")

    set_parser = subparsers.add_parser("set", help="在线修改视觉参数")
    set_parser.add_argument("assignments", nargs="+", metavar="名称=数值")

    subparsers.add_parser("pid-get", help="读取STM32当前RAM控制参数")
    pid_set = subparsers.add_parser("pid-set", help="在线修改STM32 RAM参数")
    pid_set.add_argument("assignments", nargs="+", metavar="名称=数值")
    subparsers.add_parser("pid-reset", help="恢复本次固件源码默认参数")
    pid_test = subparsers.add_parser("pid-test", help="运行固定阶跃测试")
    pid_test.add_argument("assignments", nargs="+", metavar="名称=数值")
    subparsers.add_parser("pid-stop", help="停止阶跃测试并停车")

    pid_auto = subparsers.add_parser(
        "pid-auto", help="自动执行阶跃、评分和坐标搜索"
    )
    pid_auto.add_argument(
        "--stage", choices=("inner", "outer"), required=True
    )
    pid_auto.add_argument("--rounds", type=int, default=1)
    pid_auto.add_argument("--duration", type=float, default=2.5)
    pid_auto.add_argument(
        "--target",
        type=float,
        default=None,
        help="inner单位deg，outer单位px；默认分别为2和50",
    )

    mark_parser = subparsers.add_parser("mark", help="标记实验阶段")
    mark_parser.add_argument("label")

    subparsers.add_parser("snapshot", help="保存当前画面")
    subparsers.add_parser("stop", help="安全结束本次实验")

    sync_parser = subparsers.add_parser(
        "sync", help="微调画面和标注的时间对齐"
    )
    sync_parser.add_argument("milliseconds", type=float)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.command == "pid-auto" and args.target is None:
        args.target = 2.0 if args.stage == "inner" else 50.0
    if args.command == "status":
        try:
            result = _request(args.url, "/status")
        except RuntimeError:
            if not args.status_file.is_file():
                raise
            result = json.loads(
                args.status_file.read_text(encoding="utf-8")
            )
            result["_source"] = "final_status_file"
    elif args.command == "report":
        if not args.status_file.is_file():
            raise RuntimeError("还没有实验状态文件")
        status = json.loads(
            args.status_file.read_text(encoding="utf-8")
        )
        report_path = status.get("analysis_report")
        if not report_path or not Path(report_path).is_file():
            raise RuntimeError("最近一次实验尚无分析报告")
        print(Path(report_path).read_text(encoding="utf-8"), end="")
        return 0
    elif args.command == "set":
        result = _request(
            args.url,
            "/config",
            {"params": _assignments(args.assignments)},
        )
    elif args.command == "pid-get":
        result = _submit(args.url, "/pid")
    elif args.command == "pid-set":
        result = _submit(
            args.url,
            "/pid/set",
            {"params": _pid_assignments(args.assignments)},
        )
    elif args.command == "pid-reset":
        result = _submit(args.url, "/pid/reset")
    elif args.command == "pid-test":
        result = _submit(
            args.url, "/pid/test", _parse_pid_test(args.assignments)
        )
    elif args.command == "pid-stop":
        result = _submit(args.url, "/pid/stop")
    elif args.command == "pid-auto":
        result = _run_pid_auto(args)
    elif args.command == "mark":
        result = _request(args.url, "/mark", {"label": args.label})
    elif args.command == "snapshot":
        result = _request(args.url, "/snapshot", {})
    elif args.command == "stop":
        result = _request(args.url, "/stop", {})
    elif args.command == "sync":
        result = _request(
            args.url,
            "/sync",
            {"offset_ms": args.milliseconds},
        )
    else:
        raise AssertionError(args.command)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print("ERROR:", exc, file=sys.stderr)
        raise SystemExit(1)
