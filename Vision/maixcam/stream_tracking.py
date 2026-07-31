"""Competition mode: track the ball, stream H.264, and emit UDP telemetry."""

import os
import sys

from maix import app, camera, err, image, rtsp, time

import ball_config as cfg
from main import (
    build_detector,
    build_pipe_detector,
    build_tracker,
    configure_camera,
    draw_overlay,
    init_display,
    init_uart,
    process_frame,
)
from network_link import UdpVisionLink
from loop_timing import periodic_due
from stm32_link import Stm32Link
from stream_protocol import (
    config_snapshot,
    make_status_packet,
    make_stm32_feedback_packet,
    make_tracking_packet,
)


def _session_id():
    return "{:08x}".format(time.ticks_ms() & 0xFFFFFFFF)


def start_rtsp(stream_camera):
    server = rtsp.Rtsp(
        "",
        cfg.STREAM_RTSP_PORT,
        cfg.STREAM_FPS,
        rtsp.RtspStreamType.RTSP_STREAM_H264,
        cfg.STREAM_BITRATE,
    )
    err.check_raise(
        server.bind_camera(stream_camera), "failed to bind RTSP camera"
    )
    err.check_raise(server.start(), "failed to start RTSP server")
    try:
        urls = server.get_urls()
        url = urls[0] if urls else server.get_url()
    except Exception:
        url = server.get_url()
    print("RTSP stream:", url)
    return server, url


def _status_packet(
    session_id,
    sequence,
    device_ms,
    state,
    rtsp_url,
    link,
    detector,
    tracker,
):
    if tracker is None and hasattr(detector, "pipe"):
        roi = detector.pipe.roi
        axis_start = detector.pipe.axis.start.tuple()
        axis_end = detector.pipe.axis.end.tuple()
    else:
        roi = detector.full_roi
        axis_start = tracker.axis_start
        axis_end = tracker.axis_end
    return make_status_packet(
        session_id=session_id,
        sequence=sequence,
        device_ms=device_ms,
        state=state,
        rtsp_url=rtsp_url,
        control_port=cfg.STREAM_CONTROL_PORT,
        camera_size=(cfg.CAMERA_WIDTH, cfg.CAMERA_HEIGHT),
        stream_size=(cfg.STREAM_WIDTH, cfg.STREAM_HEIGHT),
        stream_fps=cfg.STREAM_FPS,
        stream_bitrate=cfg.STREAM_BITRATE,
        roi=roi,
        axis_start=axis_start,
        axis_end=axis_end,
        current_config=config_snapshot(detector, tracker),
        network_errors=link.send_errors,
        control_errors=link.control_errors,
    )


def main():
    stm32_link = Stm32Link(
        init_uart(),
        output_scale=cfg.CONTROL_OUTPUT_SCALE,
    )
    screen = init_display() if cfg.STREAM_LOCAL_PREVIEW else None
    cam = camera.Camera(
        width=cfg.CAMERA_WIDTH,
        height=cfg.CAMERA_HEIGHT,
        format=image.Format.FMT_RGB888,
        fps=cfg.CAMERA_FPS,
        buff_num=cfg.CAMERA_BUFFER_COUNT,
    )
    stream_cam = cam.add_channel(
        width=cfg.STREAM_WIDTH,
        height=cfg.STREAM_HEIGHT,
        format=image.Format.FMT_YVU420SP,
        fps=cfg.STREAM_FPS,
        buff_num=cfg.STREAM_BUFFER_COUNT,
    )
    configure_camera(cam)

    detector = build_detector()
    tracker = build_tracker()
    pipe_detector = build_pipe_detector()
    session_id = _session_id()
    server = None
    link = None

    try:
        server, rtsp_url = start_rtsp(stream_cam)
        link = UdpVisionLink(
            session_id=session_id,
            telemetry_targets=cfg.STREAM_TELEMETRY_TARGETS,
            telemetry_port=cfg.STREAM_TELEMETRY_PORT,
            control_port=cfg.STREAM_CONTROL_PORT,
            control_token=cfg.STREAM_CONTROL_TOKEN,
        )

        uart_period_ms = max(1, int(1000 / cfg.TELEMETRY_HZ))
        loop_period_ms = (
            max(1, int(1000 / cfg.CONTROL_LOOP_HZ))
            if cfg.CONTROL_LOOP_HZ > 0
            else None
        )
        network_period_ms = max(
            1, int(1000 / cfg.STREAM_TELEMETRY_HZ)
        )
        status_period_ms = max(1, int(1000 / cfg.STREAM_STATUS_HZ))
        console_period_ms = max(1, int(1000 / cfg.CONSOLE_HZ))
        preview_period_ms = max(
            1, int(1000 / cfg.STREAM_LOCAL_PREVIEW_HZ)
        )

        start_ms = time.ticks_ms()
        last_frame_ms = start_ms
        next_uart_ms = start_ms
        next_loop_ms = start_ms
        next_network_ms = start_ms
        next_status_ms = start_ms
        last_console_ms = 0
        last_preview_ms = 0
        frame_id = 0
        sequence = 0
        camera_errors = 0
        measured_window = 0
        valid_window = 0
        frame_window = 0
        detect_window_ms = 0
        measured_ratio = 0.0

        while True:
            if app.need_exit():
                break
            stm32_link.poll_commands()
            feedback_device_ms = time.ticks_ms() - start_ms
            for feedback in stm32_link.drain_feedback():
                link.send(
                    make_stm32_feedback_packet(
                        session_id,
                        sequence,
                        feedback_device_ms,
                        feedback,
                    )
                )
                sequence += 1
            for event in link.poll_controls(detector, tracker, cfg):
                if event["ok"]:
                    print(
                        "runtime config applied:",
                        event["request_id"],
                        event["applied"],
                    )
                else:
                    print(
                        "runtime config rejected:",
                        event["request_id"],
                        event["errors"],
                    )

            try:
                img = cam.read()
                camera_errors = 0
            except Exception as exc:
                camera_errors += 1
                if camera_errors <= cfg.STREAM_CAMERA_RETRIES:
                    print(
                        "stream camera read retry {}/{}: {}".format(
                            camera_errors,
                            cfg.STREAM_CAMERA_RETRIES,
                            exc,
                        )
                    )
                    time.sleep_ms(5)
                    continue
                print("stream camera failed repeatedly; stopping")
                break

            now_ms = time.ticks_ms()
            device_ms = now_ms - start_ms
            loop_dt_ms = now_ms - last_frame_ms
            last_frame_ms = now_ms
            detect_start_ms = time.ticks_ms()
            detection, state = process_frame(
                img,
                now_ms,
                frame_id,
                detector,
                tracker,
                pipe_detector,
            )
            detect_ms = time.ticks_ms() - detect_start_ms
            fps_value = time.fps()

            frame_window += 1
            measured_window += 1 if state["measured"] else 0
            valid_window += 1 if state["valid"] else 0
            detect_window_ms += detect_ms

            uart_due, next_uart_ms = periodic_due(
                now_ms, next_uart_ms, uart_period_ms
            )
            if uart_due:
                stm32_link.send_state(state)

            network_due, next_network_ms = periodic_due(
                now_ms, next_network_ms, network_period_ms
            )
            if network_due:
                packet = make_tracking_packet(
                    session_id,
                    sequence,
                    device_ms,
                    frame_id,
                    loop_dt_ms,
                    fps_value,
                    detect_ms,
                    state,
                    detection,
                )
                link.send(packet)
                sequence += 1

            status_due, next_status_ms = periodic_due(
                now_ms, next_status_ms, status_period_ms
            )
            if status_due:
                link.send(
                    _status_packet(
                        session_id,
                        sequence,
                        device_ms,
                        "running",
                        rtsp_url,
                        link,
                        detector,
                        tracker,
                    )
                )
                sequence += 1

            if now_ms - last_console_ms >= console_period_ms:
                measured_ratio = (
                    float(measured_window) / frame_window
                    if frame_window
                    else 0.0
                )
                valid_ratio = (
                    float(valid_window) / frame_window
                    if frame_window
                    else 0.0
                )
                average_detect_ms = (
                    float(detect_window_ms) / frame_window
                    if frame_window
                    else 0.0
                )
                print(
                    "STREAM fps={:.1f} measured={:.1f}% valid={:.1f}% "
                    "detect_ms={:.2f} pipe={} age={} "
                    "net_err={} ctrl_err={}".format(
                        fps_value,
                        measured_ratio * 100.0,
                        valid_ratio * 100.0,
                        average_detect_ms,
                        "OK" if detection["pipe"]["valid"] else "STALE",
                        detection["pipe"]["age_frames"],
                        link.send_errors,
                        link.control_errors,
                    )
                )
                frame_window = 0
                measured_window = 0
                valid_window = 0
                detect_window_ms = 0
                last_console_ms = now_ms

            if (
                screen is not None
                and now_ms - last_preview_ms >= preview_period_ms
            ):
                draw_overlay(
                    img, detection, state, fps_value, measured_ratio
                )
                screen.show(img)
                last_preview_ms = now_ms

            frame_id += 1
            if loop_period_ms is not None:
                pace_now_ms = time.ticks_ms()
                _, next_loop_ms = periodic_due(
                    pace_now_ms,
                    next_loop_ms,
                    loop_period_ms,
                )
                remaining_ms = next_loop_ms - time.ticks_ms()
                if remaining_ms > 0:
                    time.sleep_ms(remaining_ms)
    finally:
        exit_type, exit_value, _ = sys.exc_info()
        exit_code = 0
        if exit_type is SystemExit:
            requested_code = getattr(exit_value, "code", None)
            if requested_code is None:
                requested_args = getattr(exit_value, "args", ())
                requested_code = requested_args[0] if requested_args else 0
            exit_code = requested_code if isinstance(requested_code, int) else 1
        elif exit_type is not None:
            print(
                "stream cleanup: unexpected exception",
                exit_type.__name__,
                repr(exit_value),
            )
            exit_code = 1
        if link is not None:
            try:
                now_ms = time.ticks_ms()
                link.send(
                    _status_packet(
                        session_id,
                        0,
                        0,
                        "stopping",
                        rtsp_url,
                        link,
                        detector,
                        tracker,
                    )
                )
            except Exception:
                pass
        # On the current MaixCAM firmware, explicitly calling Rtsp.stop()
        # after MaixVision has raised app.need_exit() triggers a native
        # double-release SIGSEGV.  Let the application runtime own RTSP and
        # multimedia-driver teardown during process exit.
        if link is not None:
            link.close()
        print("stream mode stopped")
        if server is not None:
            # After an RTSP client has connected, this firmware also crashes
            # in the Rtsp C++ destructor when Python unwinds.  All application
            # resources above are already closed; process exit lets Linux
            # reclaim the multimedia handles without invoking that destructor.
            try:
                sys.stdout.flush()
            except Exception:
                pass
            os._exit(exit_code)


if __name__ == "__main__":
    main()
