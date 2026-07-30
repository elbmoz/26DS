"""Small CLI for Codex or a teammate to control the running monitor."""

import argparse
import json
import os
from pathlib import Path
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


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
