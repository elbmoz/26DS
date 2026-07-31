"""Windows competition monitor for MaixCAM RTSP and UDP telemetry."""

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2

from iteration_bridge import IterationBridge
from preview_overlay import STATUS_BAR_HEIGHT, build_preview_frame
from receiver_core import (
    FfmpegPipeline,
    SessionLogger,
    TelemetryReceiver,
    normalize_rtsp_url,
    parse_parameter_assignments,
    unique_session_directory,
)


WINDOW_NAME = "Pipe Ball Competition Stream"
LIVE_STATUS_INTERVAL_S = 0.10
TUNING_POLL_INTERVAL_S = 0.05
# Browser preview encoding runs on Windows and is deliberately independent of
# MaixCAM detection.  Twenty frames per second keeps motion readable while
# leaving the 30 FPS decoder/logger path untouched.
BRIDGE_PREVIEW_INTERVAL_S = 1.0 / 20.0


class _StopRequested(Exception):
    pass


def build_parser():
    default_output = (
        Path(__file__).resolve().parents[1]
        / "captures"
        / "stream_sessions"
    )
    parser = argparse.ArgumentParser(
        description=(
            "Discover MaixCAM telemetry, show a low-latency synchronized "
            "preview, record on Windows, and save experiment logs."
        )
    )
    parser.add_argument("--rtsp-url")
    parser.add_argument(
        "--device-ip",
        default=os.environ.get("MAIXCAM_IP", "10.16.6.1"),
    )
    parser.add_argument("--control-port", type=int, default=42102)
    parser.add_argument("--telemetry-port", type=int, default=42101)
    parser.add_argument("--discovery-timeout", type=float, default=15.0)
    parser.add_argument("--output-root", type=Path, default=default_output)
    parser.add_argument("--transport", choices=("tcp", "udp"), default="tcp")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-record", action="store_true")
    parser.add_argument("--no-remux-mp4", action="store_true")
    parser.add_argument(
        "--sync-offset-ms",
        type=float,
        default=0.0,
        help=(
            "fine adjustment added when matching video timestamps to UDP "
            "telemetry; normally leave at 0"
        ),
    )
    parser.add_argument(
        "--iteration-port",
        type=int,
        default=int(os.environ.get("VISION_ITERATION_PORT", "8765")),
        help="localhost API port used by iteration_client.py",
    )
    parser.add_argument(
        "--no-iteration-api",
        action="store_true",
        help="disable localhost status/control API and live_status.json",
    )
    parser.add_argument(
        "--token",
        default=os.environ.get("PIPE_BALL_CONTROL_TOKEN", "pipe-ball-local"),
    )
    parser.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="apply a validated runtime vision parameter after discovery",
    )
    return parser


def _raise_if_stop_requested(bridge):
    if bridge is not None and bridge.stop_requested():
        raise _StopRequested()


def _wait_for_status(receiver, timeout, bridge):
    deadline = time.monotonic() + max(0.0, float(timeout))
    while True:
        _raise_if_stop_requested(bridge)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        status = receiver.wait_for_status(min(0.25, remaining))
        if status is not None:
            return status


def _subscribe_until_ack(
    receiver,
    device_ip,
    control_port,
    token,
    timeout,
    bridge,
    retry_interval=1.0,
):
    """Retry subscription while the MaixCAM finishes booting its app."""
    deadline = time.monotonic() + max(0.0, float(timeout))
    retry_interval = max(0.1, float(retry_interval))
    while True:
        _raise_if_stop_requested(bridge)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return None
        request_id = receiver.subscribe(device_ip, control_port, token)
        ack = receiver.wait_for_ack(
            request_id,
            timeout=min(0.5, remaining),
        )
        _raise_if_stop_requested(bridge)
        if ack is not None:
            return ack

        retry_at = min(deadline, time.monotonic() + retry_interval)
        while time.monotonic() < retry_at:
            _raise_if_stop_requested(bridge)
            time.sleep(min(0.1, retry_at - time.monotonic()))


def _read_latest_frame(pipeline, timeout, bridge):
    deadline = time.monotonic() + max(0.0, float(timeout))
    while True:
        _raise_if_stop_requested(bridge)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None, None
        frame, frame_info = pipeline.read_latest_frame(
            timeout=min(0.25, remaining)
        )
        if frame is not None:
            return frame, frame_info
        if pipeline.returncode is not None or pipeline.reader_error is not None:
            return None, None


def _save_snapshot(session_dir, frame):
    path = session_dir / "snapshot_{}.jpg".format(
        time.strftime("%H%M%S")
    )
    cv2.imwrite(str(path), frame)
    return path


def _tracking_summary(tracking):
    if not tracking:
        return None
    return {
        "session": tracking.get("session"),
        "seq": tracking.get("seq"),
        "device_ms": tracking.get("device_ms"),
        "frame_id": tracking.get("frame_id"),
        "measured": bool(tracking.get("measured")),
        "valid": bool(tracking.get("valid")),
        "position": tracking.get("position"),
        "position_px": tracking.get("position_px"),
        "travel_position_px": tracking.get("travel_position_px"),
        "travel_length_px": tracking.get("travel_length_px"),
        "target_axis_px": tracking.get("target_axis_px"),
        "error_px": tracking.get("error_px"),
        "lateral_px": tracking.get("lateral_px"),
        "velocity_px_s": tracking.get("velocity_px_s"),
        "quality": tracking.get("quality"),
        "coasting": bool(tracking.get("coasting")),
        "detect_ms": tracking.get("detect_ms"),
        "fps": tracking.get("fps"),
        "pipe_valid": bool(tracking.get("pipe_valid")),
        "pipe_age_frames": tracking.get("pipe_age_frames"),
        "axis_x0": tracking.get("axis_x0"),
        "axis_y0": tracking.get("axis_y0"),
        "axis_x1": tracking.get("axis_x1"),
        "axis_y1": tracking.get("axis_y1"),
        "q9": tracking.get("q9"),
    }


def _feedback_summary(feedback):
    if not feedback:
        return None
    fields = (
        "session",
        "transport_seq",
        "device_ms",
        "seq",
        "seq_gap",
        "mcu_ms",
        "vision_frame",
        "vision_age_ms",
        "position_px",
        "velocity_px_s",
        "control_error_px",
        "p_term",
        "i_term",
        "d_term",
        "motor_command",
        "motor_status",
        "motor_status_name",
        "feedback_version",
        "target_rod_angle_deg",
        "actual_rod_angle_deg",
        "rod_rate_deg_s",
        "angle_error_deg",
        "desired_motor_speed",
        "position_age_ms",
        "position_valid",
        "protection_state",
        "tuning_mode",
    )
    return {name: feedback.get(name) for name in fields}


def _run_session_analysis(session_dir, video_path=None):
    analyzer = (
        Path(__file__).resolve().parents[1]
        / "tools"
        / "analyze_tracking_log.py"
    )
    tracking_csv = Path(session_dir) / "tracking.csv"
    if not analyzer.is_file() or not tracking_csv.is_file():
        return None
    with tracking_csv.open(encoding="utf-8") as source:
        source.readline()
        if not source.readline():
            return None
    command = [sys.executable, str(analyzer), str(tracking_csv)]
    if video_path is not None and Path(video_path).is_file():
        command.extend(["--video", str(video_path)])
    analysis_json = Path(session_dir) / "analysis.json"
    report_path = Path(session_dir) / "analysis.txt"
    command.extend(
        [
            "--json-out",
            str(analysis_json),
            "--text-out",
            str(report_path),
        ]
    )
    result = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
    )
    if not report_path.is_file():
        report = result.stdout
        if result.stderr:
            report += "\n" + result.stderr
        report_path.write_text(report.strip() + "\n", encoding="utf-8")
    return report_path if result.returncode == 0 else None


def main(argv=None):
    args = build_parser().parse_args(argv)
    startup_params = parse_parameter_assignments(args.set)
    session_dir = unique_session_directory(args.output_root)
    logger = SessionLogger(session_dir)
    receiver = TelemetryReceiver(args.telemetry_port, logger)
    runtime_dir = Path(__file__).resolve().parents[1] / "runtime"
    pipeline = None
    tuning = None
    bridge = None
    latest_status = None
    current_display_frame = None
    final_video = None
    analysis_report = None
    sync_offset_ms = float(args.sync_offset_ms)
    pending_configs = {}
    pending_pids = {}
    latest_pid_config = None
    last_command = None
    manifest = {
        "status": "starting",
        "session_directory": str(session_dir),
        "recording_enabled": not args.no_record,
        "transport": args.transport,
        "initial_sync_offset_ms": sync_offset_ms,
    }

    print("session:", session_dir)
    print("waiting for UDP telemetry on port", args.telemetry_port)
    if not args.no_iteration_api:
        try:
            bridge = IterationBridge(
                session_dir,
                runtime_dir,
                port=args.iteration_port,
            )
            bridge.start()
            receiver.add_tracking_listener(
                lambda packet, live_bridge=bridge: (
                    live_bridge.publish_telemetry(
                        {
                            "schema": 1,
                            "host_epoch_ns": int(
                                packet.get("_host_epoch_ns")
                                or time.time_ns()
                            ),
                            "tracking": _tracking_summary(packet),
                        }
                    )
                )
            )
            receiver.add_feedback_listener(
                lambda packet, live_bridge=bridge: (
                    live_bridge.publish_telemetry(
                        {
                            "schema": 1,
                            "host_epoch_ns": int(
                                packet.get("_host_epoch_ns")
                                or time.time_ns()
                            ),
                            "feedback": _feedback_summary(packet),
                        }
                    )
                )
            )
            bridge.publish(
                {"state": "discovering_telemetry"},
                force=True,
            )
            manifest["iteration_api_url"] = bridge.status().get("api_url")
            print("iteration API:", bridge.status().get("api_url"))
            print("live status:", bridge.live_status_path)
        except OSError as exc:
            bridge = None
            logger.log_event(
                "iteration_api_unavailable", {"error": str(exc)}
            )
            print("iteration API unavailable:", exc)

    receiver.start()
    try:
        if args.device_ip:
            subscribe_ack = _subscribe_until_ack(
                receiver,
                args.device_ip,
                args.control_port,
                args.token,
                timeout=args.discovery_timeout,
                bridge=bridge,
            )
            if subscribe_ack and subscribe_ack.get("ok"):
                print(
                    "telemetry subscription established:",
                    args.device_ip,
                )
            else:
                print(
                    "telemetry subscription has no ACK; "
                    "continuing with broadcast discovery"
                )

        if args.rtsp_url:
            device_ip = args.device_ip or "10.16.6.1"
            rtsp_url = normalize_rtsp_url(args.rtsp_url, device_ip)
            latest_status = _wait_for_status(
                receiver,
                min(2.0, args.discovery_timeout),
                bridge,
            )
        else:
            latest_status = _wait_for_status(
                receiver,
                args.discovery_timeout,
                bridge,
            )
            if latest_status is None:
                if not args.device_ip:
                    raise RuntimeError(
                        "no telemetry discovered; check Windows firewall or "
                        "pass --device-ip and --rtsp-url"
                    )
                device_ip = args.device_ip
                rtsp_url = normalize_rtsp_url(None, device_ip)
            else:
                device_ip = latest_status["_source_ip"]
                rtsp_url = normalize_rtsp_url(
                    latest_status.get("rtsp_url"), device_ip
                )

        if latest_status is not None:
            stream_size = latest_status.get("stream_size") or (448, 336)
            stream_fps = int(latest_status.get("stream_fps") or 30)
        else:
            stream_size = (448, 336)
            stream_fps = 30
        width, height = [int(value) for value in stream_size]
        print("device:", device_ip)
        print("RTSP:", rtsp_url)
        if bridge is not None:
            bridge.publish(
                {
                    "state": "connecting_video",
                    "device_ip": device_ip,
                    "rtsp_url": rtsp_url,
                },
                force=True,
            )

        if startup_params:
            request_id = receiver.send_parameters(
                args.token, startup_params
            )
            ack = receiver.wait_for_ack(request_id, timeout=2.0)
            _raise_if_stop_requested(bridge)
            if not ack or not ack.get("ok"):
                raise RuntimeError(
                    "startup parameter update rejected: {}".format(ack)
                )
            print("parameters applied:", ack.get("applied"))

        pipeline = FfmpegPipeline(
            rtsp_url,
            width,
            height,
            stream_fps,
            session_dir,
            record=not args.no_record,
            transport=args.transport,
            remux_mp4=not args.no_remux_mp4,
        )
        pipeline.start()
        manifest.update(
            {
                "status": "waiting_for_first_frame",
                "device_ip": device_ip,
                "rtsp_url": rtsp_url,
                "stream_width": width,
                "stream_height": height,
                "stream_fps": stream_fps,
                "ffmpeg_started_epoch_ns": pipeline.started_epoch_ns,
                "ffmpeg_started_monotonic_ns": (
                    pipeline.started_monotonic_ns
                ),
            }
        )
        logger.log_event("pipeline_started", manifest)
        if bridge is not None:
            bridge.publish(
                {
                    "state": "waiting_for_first_frame",
                    "device_ip": device_ip,
                },
                force=True,
            )

        if not args.headless:
            from tuning_panel import TuningPanel

            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                WINDOW_NAME,
                width * 2,
                (height + STATUS_BAR_HEIGHT) * 2,
            )
            initial_config = (
                latest_status.get("config", {}) if latest_status else {}
            )
            tuning = TuningPanel(
                initial_config,
                sync_offset_ms=sync_offset_ms,
            )
            print(
                "keys: Q/ESC stop, P apply tuning, S snapshot; "
                "the Chinese tuning window also has an apply button"
            )

        frame_id = 0
        first_frame_monotonic = None
        last_live_publish = 0.0
        last_bridge_preview = 0.0
        last_tuning_poll = 0.0
        stop_requested = False

        def submit_parameters(params, source, command_id=None):
            nonlocal last_command
            if not isinstance(params, dict) or not params:
                raise ValueError("config params must be a non-empty object")
            request_id = receiver.send_parameters(args.token, params)
            pending_configs[request_id] = {
                "source": str(source),
                "command_id": command_id,
                "params": dict(params),
                "sent_monotonic": time.monotonic(),
            }
            last_command = {
                "id": command_id or request_id,
                "type": "set_config",
                "state": "waiting_for_device_ack",
                "params": dict(params),
            }
            if tuning is not None and source == "tuning_panel":
                tuning.set_status("已发送，等待设备确认…")
            return request_id

        def submit_pid(action, params=None, command_id=None):
            nonlocal last_command
            request_id = receiver.send_pid_request(
                args.token, action, params
            )
            pending_pids[request_id] = {
                "command_id": command_id,
                "action": str(action),
                "params": dict(params or {}),
                "sent_monotonic": time.monotonic(),
            }
            last_command = {
                "id": command_id or request_id,
                "type": "pid_{}".format(action),
                "state": "waiting_for_stm32_ack",
                "params": dict(params or {}),
            }
            return request_id

        while not stop_requested:
            frame, frame_info = _read_latest_frame(
                pipeline,
                timeout=(
                    max(8.0, args.discovery_timeout)
                    if first_frame_monotonic is None
                    else 3.0
                ),
                bridge=bridge,
            )
            if frame is None:
                raise RuntimeError(
                    "FFmpeg produced no new frame (return code {}, "
                    "reader error {})".format(
                        pipeline.returncode,
                        repr(pipeline.reader_error),
                    )
                )
            if first_frame_monotonic is None:
                first_frame_monotonic = time.monotonic()
                manifest["status"] = "running"
                manifest["first_video_frame_monotonic_ns"] = (
                    time.monotonic_ns()
                )

            status = receiver.status_snapshot()
            if status is not None:
                latest_status = status

            if tuning is not None:
                sync_offset_ms = tuning.sync_offset_ms()
            tracking, match_info = receiver.tracking_for_epoch(
                frame_info.get("source_epoch_ns"),
                sync_offset_ms=sync_offset_ms,
            )
            sync_info = dict(match_info)
            sync_info.update(
                {
                    "video_latency_ms": frame_info.get(
                        "pipeline_latency_ms"
                    ),
                    "dropped_frames": pipeline.dropped_preview_frames,
                }
            )
            logger.log_video_frame(
                frame_id,
                tracking,
                frame_info=frame_info,
                match_info=match_info,
            )
            frame_id += 1

            if not args.headless or bridge is not None:
                current_display_frame = build_preview_frame(
                    frame,
                    tracking,
                    latest_status,
                    recording=not args.no_record,
                    sync_info=sync_info,
                )
            else:
                current_display_frame = frame

            if not args.headless:
                cv2.imshow(WINDOW_NAME, current_display_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    snapshot = _save_snapshot(
                        session_dir, current_display_frame
                    )
                    logger.log_event("snapshot", {"path": str(snapshot)})
                    print("snapshot:", snapshot)
                if tuning is not None:
                    tuning_poll_time = time.monotonic()
                    if (
                        tuning_poll_time - last_tuning_poll
                        >= TUNING_POLL_INTERVAL_S
                    ):
                        last_tuning_poll = tuning_poll_time
                        if not tuning.poll():
                            break
                    if key == ord("p") or tuning.consume_apply_request():
                        submit_parameters(
                            tuning.read(), "tuning_panel"
                        )
            if bridge is not None:
                now_preview = time.monotonic()
                if (
                    now_preview - last_bridge_preview
                    >= BRIDGE_PREVIEW_INTERVAL_S
                ):
                    encoded, jpeg = cv2.imencode(
                        ".jpg",
                        current_display_frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 84],
                    )
                    if encoded:
                        bridge.publish_frame(jpeg.tobytes())
                        # Keep an accumulated deadline instead of resetting
                        # to ``now``.  With a 30 FPS decoder, resetting here
                        # quantizes a requested 20 FPS preview down to 15 FPS
                        # (one publish every two decoded frames).
                        if last_bridge_preview <= 0.0:
                            last_bridge_preview = now_preview
                        else:
                            last_bridge_preview += (
                                BRIDGE_PREVIEW_INTERVAL_S
                            )
                            if (
                                last_bridge_preview
                                < now_preview
                                - BRIDGE_PREVIEW_INTERVAL_S
                            ):
                                last_bridge_preview = now_preview

            for request_id in list(pending_pids):
                request = pending_pids[request_id]
                applied_ack = receiver.pop_ack(request_id)
                if applied_ack is not None:
                    pending_pids.pop(request_id, None)
                    ok = bool(applied_ack.get("ok"))
                    if ok:
                        latest_pid_config = dict(
                            applied_ack.get("config", {})
                        )
                    logger.log_event(
                        "pid_result",
                        {
                            "request_id": request_id,
                            "command_id": request["command_id"],
                            "ack": applied_ack,
                        },
                    )
                    last_command = {
                        "id": request["command_id"] or request_id,
                        "type": "pid_{}".format(request["action"]),
                        "state": "applied" if ok else "rejected",
                        "ack": applied_ack,
                    }
                elif (
                    time.monotonic() - request["sent_monotonic"] > 4.0
                ):
                    pending_pids.pop(request_id, None)
                    last_command = {
                        "id": request["command_id"] or request_id,
                        "type": "pid_{}".format(request["action"]),
                        "state": "ack_timeout",
                    }
                    logger.log_event(
                        "pid_ack_timeout", {"request_id": request_id}
                    )

            for request_id in list(pending_configs):
                request = pending_configs[request_id]
                applied_ack = receiver.pop_ack(request_id)
                if applied_ack is not None:
                    pending_configs.pop(request_id, None)
                    ok = bool(applied_ack.get("ok"))
                    logger.log_event(
                        "config_result",
                        {
                            "request_id": request_id,
                            "source": request["source"],
                            "command_id": request["command_id"],
                            "ack": applied_ack,
                        },
                    )
                    last_command = {
                        "id": request["command_id"] or request_id,
                        "type": "set_config",
                        "state": "applied" if ok else "rejected",
                        "ack": applied_ack,
                    }
                    if ok:
                        print(
                            "parameters applied:",
                            applied_ack.get("applied"),
                        )
                        if tuning is not None:
                            tuning.acknowledge(
                                applied_ack.get("config", {})
                            )
                        if latest_status is not None:
                            latest_status["config"] = applied_ack.get(
                                "config", {}
                            )
                    else:
                        print(
                            "parameter update rejected:", applied_ack
                        )
                        if tuning is not None:
                            tuning.set_status(
                                "设备拒绝参数，请查看终端", ok=False
                            )
                elif (
                    time.monotonic() - request["sent_monotonic"] > 3.0
                ):
                    pending_configs.pop(request_id, None)
                    last_command = {
                        "id": request["command_id"] or request_id,
                        "type": "set_config",
                        "state": "ack_timeout",
                    }
                    logger.log_event(
                        "config_ack_timeout",
                        {
                            "request_id": request_id,
                            "source": request["source"],
                        },
                    )
                    if tuning is not None:
                        tuning.set_status("设备确认超时", ok=False)

            if bridge is not None:
                for command in bridge.poll_commands():
                    command_id = command["id"]
                    command_type = command["type"]
                    body = command["body"]
                    try:
                        if command_type == "set_config":
                            submit_parameters(
                                body.get("params"),
                                "iteration_api",
                                command_id=command_id,
                            )
                        elif command_type == "pid_get":
                            submit_pid("get", command_id=command_id)
                        elif command_type == "pid_set":
                            params = body.get("params")
                            if not isinstance(params, dict) or not params:
                                raise ValueError(
                                    "PID params must be a non-empty object"
                                )
                            submit_pid(
                                "set", params, command_id=command_id
                            )
                        elif command_type == "pid_reset":
                            submit_pid("reset", command_id=command_id)
                        elif command_type == "pid_test":
                            submit_pid(
                                "test",
                                {
                                    "mode": body.get("mode"),
                                    "target": body.get("target"),
                                    "duration_ms": body.get("duration_ms"),
                                },
                                command_id=command_id,
                            )
                        elif command_type == "pid_stop":
                            submit_pid("stop", command_id=command_id)
                        elif command_type == "mark":
                            label = str(body.get("label", "")).strip()
                            if not label:
                                raise ValueError("marker label is required")
                            label = label[:120]
                            logger.log_event(
                                "experiment_marker",
                                {
                                    "command_id": command_id,
                                    "label": label,
                                },
                            )
                            print("experiment marker:", label)
                            last_command = {
                                "id": command_id,
                                "type": "mark",
                                "state": "completed",
                                "label": label,
                            }
                        elif command_type == "snapshot":
                            if current_display_frame is None:
                                raise RuntimeError(
                                    "no video frame is available yet"
                                )
                            snapshot = _save_snapshot(
                                session_dir, current_display_frame
                            )
                            logger.log_event(
                                "snapshot",
                                {
                                    "command_id": command_id,
                                    "path": str(snapshot),
                                },
                            )
                            last_command = {
                                "id": command_id,
                                "type": "snapshot",
                                "state": "completed",
                                "path": str(snapshot),
                            }
                        elif command_type == "set_sync_offset":
                            requested_offset = float(
                                body.get("offset_ms", 0.0)
                            )
                            sync_offset_ms = max(
                                -1000.0, min(1000.0, requested_offset)
                            )
                            if tuning is not None:
                                tuning.set_sync_offset(sync_offset_ms)
                            last_command = {
                                "id": command_id,
                                "type": "set_sync_offset",
                                "state": "completed",
                                "offset_ms": sync_offset_ms,
                            }
                            logger.log_event(
                                "sync_offset_changed", last_command
                            )
                        elif command_type == "stop":
                            stop_requested = True
                            last_command = {
                                "id": command_id,
                                "type": "stop",
                                "state": "accepted",
                            }
                        else:
                            raise ValueError(
                                "unknown command type {}".format(
                                    command_type
                                )
                            )
                    except Exception as exc:
                        last_command = {
                            "id": command_id,
                            "type": command_type,
                            "state": "rejected",
                            "error": str(exc),
                        }
                        logger.log_event(
                            "iteration_command_rejected", last_command
                        )

                now_monotonic = time.monotonic()
                if (
                    now_monotonic - last_live_publish
                    >= LIVE_STATUS_INTERVAL_S
                ):
                    bridge.publish(
                        {
                            "state": "running",
                            "device_ip": device_ip,
                            "recording": not args.no_record,
                            "uptime_s": (
                                0.0
                                if first_frame_monotonic is None
                                else round(
                                    now_monotonic
                                    - first_frame_monotonic,
                                    2,
                                )
                            ),
                            "video": {
                                "width": width,
                                "height": height,
                                "source_timestamp_ns": frame_info.get(
                                    "source_epoch_ns"
                                ),
                                "pipeline_latency_ms": frame_info.get(
                                    "pipeline_latency_ms"
                                ),
                                "decoded_frames": pipeline.decoded_frames,
                                "dropped_preview_frames": (
                                    pipeline.dropped_preview_frames
                                ),
                            },
                            "synchronization": {
                                "mode": match_info.get("mode"),
                                "matched": match_info.get("matched"),
                                "match_delta_ms": match_info.get(
                                    "match_delta_ms"
                                ),
                                "offset_ms": sync_offset_ms,
                            },
                            "tracking": _tracking_summary(tracking),
                            "feedback": _feedback_summary(
                                receiver.feedback_snapshot()
                            ),
                            "config": (
                                {}
                                if latest_status is None
                                else latest_status.get("config", {})
                            ),
                            "pending_config_requests": len(
                                pending_configs
                            ),
                            "pending_pid_requests": len(pending_pids),
                            "pid_config": latest_pid_config,
                            "last_command": last_command,
                        }
                    )
                    last_live_publish = now_monotonic

            if (
                args.duration > 0
                and time.monotonic() - first_frame_monotonic
                >= args.duration
            ):
                break

        manifest["status"] = (
            "stopped" if stop_requested else "completed"
        )
    except _StopRequested:
        manifest["status"] = "stopped"
        last_command = {
            "type": "stop",
            "state": "accepted",
        }
        return_code = 0
    except KeyboardInterrupt:
        manifest["status"] = "interrupted"
        return_code = 0
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["error"] = str(exc)
        logger.log_event("failure", {"error": str(exc)})
        print("ERROR:", exc, file=sys.stderr)
        return_code = 1
    else:
        return_code = 0
    finally:
        if bridge is not None:
            bridge.publish({"state": "stopping"}, force=True)
        if tuning is not None:
            tuning.close()
        if not args.headless:
            cv2.destroyAllWindows()
        if pipeline is not None:
            try:
                final_video = pipeline.stop()
                if final_video is not None:
                    manifest["video"] = str(final_video)
                elif pipeline.mkv_path.exists():
                    manifest["video"] = str(pipeline.mkv_path)
                manifest["ffmpeg_returncode"] = pipeline.returncode
                manifest["decoded_preview_frames"] = (
                    pipeline.decoded_frames
                )
                manifest["dropped_preview_frames"] = (
                    pipeline.dropped_preview_frames
                )
                manifest["timestamped_preview_frames"] = (
                    pipeline.timing_frames
                )
            except Exception as exc:
                manifest["pipeline_stop_error"] = str(exc)
        receiver.stop()
        manifest["invalid_udp_packets"] = receiver.invalid_packets
        if receiver.latest_status is not None:
            manifest["last_device_status"] = receiver.latest_status
        logger.close()
        try:
            analysis_report = _run_session_analysis(
                session_dir, final_video
            )
            if analysis_report is not None:
                manifest["analysis_report"] = str(analysis_report)
                analysis_json = session_dir / "analysis.json"
                if analysis_json.is_file():
                    manifest["analysis_json"] = str(analysis_json)
        except Exception as exc:
            manifest["analysis_error"] = str(exc)
        manifest["final_sync_offset_ms"] = sync_offset_ms
        logger.write_manifest(manifest)
        if bridge is not None:
            bridge.stop(
                {
                    "state": manifest.get("status", "completed"),
                    "video": (
                        None
                        if final_video is None
                        else str(final_video)
                    ),
                    "analysis_report": (
                        None
                        if analysis_report is None
                        else str(analysis_report)
                    ),
                    "last_command": last_command,
                }
            )
        print("saved:", session_dir)
        if analysis_report is not None:
            print("analysis:", analysis_report)
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
