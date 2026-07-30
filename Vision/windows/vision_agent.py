"""Machine-friendly upper-computer CLI for MaixCAM vision iteration."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from device_manager import (
    DeviceManagerError,
    MaixCamDeviceManager,
    preflight_source,
    tcp_reachable,
)


WINDOWS_DIR = Path(__file__).resolve().parent
VISION_DIR = WINDOWS_DIR.parent
DEFAULT_SOURCE = VISION_DIR / "maixcam"
DEFAULT_CAPTURES = VISION_DIR / "captures" / "stream_sessions"


def _manager(args):
    return MaixCamDeviceManager(
        host=args.device_ip,
        username=args.username,
        password=args.password,
        port=args.ssh_port,
        timeout=args.timeout,
    )


def _latest_session(before_names):
    if not DEFAULT_CAPTURES.is_dir():
        return None
    candidates = [
        path
        for path in DEFAULT_CAPTURES.iterdir()
        if path.is_dir() and path.name not in before_names
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def _run_experiment(args):
    with _manager(args) as manager:
        device_status = manager.status(log_lines=8)
        if not device_status["running"]:
            raise DeviceManagerError(
                "managed vision process is not running; use restart first"
            )

    DEFAULT_CAPTURES.mkdir(parents=True, exist_ok=True)
    before_names = {
        path.name for path in DEFAULT_CAPTURES.iterdir() if path.is_dir()
    }
    command = [
        sys.executable,
        str(WINDOWS_DIR / "stream_receiver.py"),
        "--device-ip",
        args.device_ip,
        "--duration",
        str(args.duration),
        "--headless",
        "--no-iteration-api",
    ]
    if args.no_record:
        command.append("--no-record")
    completed = subprocess.run(
        command,
        cwd=str(VISION_DIR.parent),
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=max(30.0, float(args.duration) + 25.0),
    )
    session = _latest_session(before_names)
    analysis = None
    if session is not None:
        analysis_path = session / "analysis.json"
        if analysis_path.is_file():
            analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    result = {
        "ok": completed.returncode == 0,
        "returncode": completed.returncode,
        "session_directory": str(session) if session else None,
        "analysis": analysis,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }
    if completed.returncode != 0:
        raise DeviceManagerError(json.dumps(result, ensure_ascii=False))
    return result


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "MaixCAM滚球视觉：部署、启停、日志、录像和自动分析"
        )
    )
    parser.add_argument(
        "--device-ip",
        default=os.environ.get("MAIXCAM_IP", "10.16.6.1"),
    )
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--username", default="root")
    parser.add_argument(
        "--password",
        default=None,
        help="默认读取 MAIXCAM_PASSWORD；MaixCAM 出厂默认 root",
    )
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help="板端视觉源码目录",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("doctor", help="检查源码、SSH、受管进程与流端口")
    deploy = sub.add_parser("deploy", help="校验并按内容版本上传")
    deploy.add_argument("--restart", action="store_true")
    sub.add_parser("start", help="启动已部署版本")
    sub.add_parser("stop", help="安全停止受管视觉进程")
    restart = sub.add_parser("restart", help="重启受管视觉进程")
    restart.add_argument(
        "--deploy", action="store_true", help="先部署本地源码"
    )
    sub.add_parser("status", help="输出机器可读设备状态")
    logs = sub.add_parser("logs", help="读取板端日志")
    logs.add_argument("--lines", type=int, default=80)
    sub.add_parser("releases", help="列出可回退版本")
    rollback = sub.add_parser("rollback", help="切换到历史版本")
    rollback.add_argument("release_id")
    rollback.add_argument("--start", action="store_true")
    experiment = sub.add_parser(
        "experiment", help="定时录制并返回自动分析 JSON"
    )
    experiment.add_argument("--duration", type=float, default=15.0)
    experiment.add_argument("--no-record", action="store_true")
    return parser


def run(args):
    if args.command == "doctor":
        preflight = preflight_source(args.source)
        with _manager(args) as manager:
            status = manager.status(log_lines=12)
        return {
            "ok": True,
            "source": {
                "release_id": preflight["release_id"],
                "python_files_checked": preflight[
                    "python_files_checked"
                ],
            },
            "device": status,
            "ports": {
                "ssh_22": tcp_reachable(
                    args.device_ip, args.ssh_port, args.timeout
                ),
                "rtsp_8554": tcp_reachable(
                    args.device_ip, 8554, 1.0
                ),
            },
        }

    if args.command == "experiment":
        return _run_experiment(args)

    with _manager(args) as manager:
        if args.command == "deploy":
            result = manager.deploy(args.source)
            if args.restart:
                result["restart"] = manager.restart()
            return result
        if args.command == "start":
            return manager.start()
        if args.command == "stop":
            return manager.stop()
        if args.command == "restart":
            result = {"ok": True}
            if args.deploy:
                result["deploy"] = manager.deploy(args.source)
            result["restart"] = manager.restart()
            return result
        if args.command == "status":
            return manager.status(log_lines=20)
        if args.command == "logs":
            return {
                "ok": True,
                "host": args.device_ip,
                "lines": args.lines,
                "log": manager.log_tail(args.lines),
            }
        if args.command == "releases":
            return {
                "ok": True,
                "current_release": manager.current_release(),
                "releases": manager.releases(),
            }
        if args.command == "rollback":
            result = manager.rollback(args.release_id)
            if args.start:
                result["start"] = manager.start()
            return result
    raise AssertionError(args.command)


def main(argv=None):
    args = build_parser().parse_args(argv)
    started = time.monotonic()
    result = run(args)
    result["command"] = args.command
    result["elapsed_ms"] = round(
        (time.monotonic() - started) * 1000.0, 1
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (
        DeviceManagerError,
        OSError,
        ValueError,
        SyntaxError,
        subprocess.TimeoutExpired,
    ) as exc:
        print(
            json.dumps(
                {
                    "ok": False,
                    "error": str(exc),
                    "error_type": type(exc).__name__,
                },
                ensure_ascii=False,
                indent=2,
            ),
            file=sys.stderr,
        )
        raise SystemExit(1)
