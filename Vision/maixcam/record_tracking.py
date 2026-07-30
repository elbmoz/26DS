"""Record a time-synchronized rolling-ball video and per-frame tracking log.

Run this file by itself in MaixVision. Move one end of the pipe by hand, then
press Stop. Each run is saved under /root/ball_tests/run_NNNN/.
"""

import os

from maix import app, camera, image, time, video

import ball_config as cfg
from ball_tracker_core import format_vision_line
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
from tracking_log import RunStats, csv_header, tracking_row
from loop_timing import periodic_due


def make_run_directory():
    root = cfg.RECORD_OUTPUT_ROOT
    try:
        os.makedirs(root)
    except OSError:
        pass

    for number in range(1, 10000):
        path = "{}/run_{:04d}".format(root, number)
        try:
            os.mkdir(path)
            return path
        except OSError:
            continue
    raise RuntimeError("no free run directory under {}".format(root))


def start_background_recorder(video_cam, video_path):
    recorder_type = getattr(video, "VideoRecorder", None)
    if recorder_type is None:
        raise RuntimeError("this MaixPy firmware has no VideoRecorder")
    # VideoRecorder defaults to open=True.  Its config_* methods are rejected
    # once the worker thread is running, so configure a closed recorder first.
    recorder = recorder_type(False)
    recorder.bind_camera(video_cam)
    recorder.config_path(video_path)
    recorder.config_resolution(
        [cfg.RECORD_VIDEO_WIDTH, cfg.RECORD_VIDEO_HEIGHT]
    )
    recorder.config_fps(cfg.RECORD_FPS)
    recorder.config_bitrate(cfg.RECORD_VIDEO_BITRATE)
    recorder.open()
    recorder.record_start()
    print(
        "background video recorder: {} fps={} bitrate={}".format(
            video_path,
            recorder.get_fps(),
            recorder.get_bitrate(),
        )
    )
    return recorder


def write_metadata(path, recorder_reported_fps, recorder_reported_bitrate):
    with open(path, "w") as output:
        output.write("format=mp4\n")
        output.write("encoder=VideoRecorder_background_thread\n")
        output.write("sync_mode=elapsed_ms_from_record_start\n")
        output.write("tracking_width={}\n".format(cfg.CAMERA_WIDTH))
        output.write("tracking_height={}\n".format(cfg.CAMERA_HEIGHT))
        output.write("video_width={}\n".format(cfg.RECORD_VIDEO_WIDTH))
        output.write("video_height={}\n".format(cfg.RECORD_VIDEO_HEIGHT))
        output.write("requested_camera_fps={}\n".format(cfg.CAMERA_FPS))
        output.write("requested_record_fps={}\n".format(cfg.RECORD_FPS))
        output.write(
            "recorder_reported_fps={}\n".format(recorder_reported_fps)
        )
        output.write(
            "requested_bitrate={}\n".format(cfg.RECORD_VIDEO_BITRATE)
        )
        output.write(
            "recorder_reported_bitrate={}\n".format(
                recorder_reported_bitrate
            )
        )
        output.write("roi={}\n".format(tuple(cfg.ROI)))
        output.write("axis_start={}\n".format(tuple(cfg.AXIS_START)))
        output.write("axis_end={}\n".format(tuple(cfg.AXIS_END)))
        output.write("target_position={}\n".format(cfg.TARGET_POSITION))
        output.write("pipe_pose_enabled={}\n".format(cfg.PIPE_POSE_ENABLED))
        output.write("pipe_search_roi={}\n".format(cfg.PIPE_SEARCH_ROI))
        output.write(
            "pipe_lab_thresholds={}\n".format(cfg.PIPE_LAB_THRESHOLDS)
        )


def main():
    run_dir = make_run_directory()
    video_path = run_dir + "/video.mp4"
    log_path = run_dir + "/tracking.csv"
    metadata_path = run_dir + "/metadata.txt"
    summary_path = run_dir + "/summary.txt"
    print("recording directory:", run_dir)

    serial_port = init_uart()
    screen = init_display()
    cam = camera.Camera(
        width=cfg.CAMERA_WIDTH,
        height=cfg.CAMERA_HEIGHT,
        format=image.Format.FMT_RGB888,
        fps=cfg.CAMERA_FPS,
        buff_num=cfg.CAMERA_BUFFER_COUNT,
    )
    video_cam = cam.add_channel(
        width=cfg.RECORD_VIDEO_WIDTH,
        height=cfg.RECORD_VIDEO_HEIGHT,
        format=image.Format.FMT_YVU420SP,
        fps=cfg.RECORD_FPS,
        buff_num=cfg.CAMERA_BUFFER_COUNT,
    )
    configure_camera(cam)

    detector = build_detector()
    tracker = build_tracker()
    pipe_detector = build_pipe_detector()
    stats = RunStats()
    send_period_ms = max(1, int(1000 / cfg.TELEMETRY_HZ))
    preview_period_ms = max(1, int(1000 / cfg.PREVIEW_HZ))
    console_period_ms = max(1, int(1000 / cfg.CONSOLE_HZ))
    start_ms = time.ticks_ms()
    last_frame_ms = start_ms
    next_send_ms = start_ms
    last_preview_ms = 0
    last_console_ms = 0
    frame_id = 0
    camera_errors = 0

    log_file = open(log_path, "w")
    log_file.write(csv_header())
    recorder = None
    try:
        recorder = start_background_recorder(video_cam, video_path)
        recorder_reported_fps = recorder.get_fps()
        recorder_reported_bitrate = recorder.get_bitrate()
        write_metadata(
            metadata_path,
            recorder_reported_fps,
            recorder_reported_bitrate,
        )
        # elapsed_ms is the authoritative video/log synchronization key.
        start_ms = time.ticks_ms()
        last_frame_ms = start_ms
        next_send_ms = start_ms
        while not app.need_exit():
            try:
                img = cam.read()
                camera_errors = 0
            except Exception as exc:
                camera_errors += 1
                print(
                    "camera read retry {}/{}: {}".format(
                        camera_errors,
                        cfg.RECORD_CAMERA_RETRIES,
                        exc,
                    )
                )
                if camera_errors >= cfg.RECORD_CAMERA_RETRIES:
                    print("camera read failed repeatedly; stopping run")
                    break
                time.sleep_ms(5)
                continue
            now_ms = time.ticks_ms()
            elapsed_ms = now_ms - start_ms
            loop_dt_ms = now_ms - last_frame_ms
            last_frame_ms = now_ms

            row_video_frame_id = int(
                elapsed_ms * cfg.RECORD_FPS / 1000
            )

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

            log_file.write(
                tracking_row(
                    frame_id,
                    row_video_frame_id,
                    elapsed_ms,
                    loop_dt_ms,
                    fps_value,
                    0,
                    state,
                    detection,
                    detect_ms,
                    detection["full_roi"],
                )
            )
            stats.update(state, detect_ms, 0)

            send_due, next_send_ms = periodic_due(
                now_ms, next_send_ms, send_period_ms
            )
            if send_due and serial_port is not None:
                serial_port.write_str(
                    format_vision_line(state, cfg.CONTROL_OUTPUT_SCALE)
                )

            if now_ms - last_console_ms >= console_period_ms:
                measured = 100.0 * stats.measured / max(1, stats.frames)
                valid = 100.0 * stats.valid / max(1, stats.frames)
                print(
                    "REC frame={} fps={:.1f} measured={:.1f}% "
                    "valid={:.1f}% detect_ms={} elapsed_ms={}".format(
                        frame_id,
                        fps_value,
                        measured,
                        valid,
                        detect_ms,
                        elapsed_ms,
                    )
                )
                last_console_ms = now_ms

            if screen is not None and now_ms - last_preview_ms >= preview_period_ms:
                measured_ratio = float(stats.measured) / max(1, stats.frames)
                draw_overlay(img, detection, state, fps_value, measured_ratio)
                img.draw_string(
                    cfg.CAMERA_WIDTH - 70,
                    8,
                    "REC",
                    image.COLOR_RED,
                    1.2,
                )
                screen.show(img)
                last_preview_ms = now_ms

            frame_id += 1
            if frame_id % max(1, cfg.RECORD_LOG_FLUSH_FRAMES) == 0:
                log_file.flush()

            if (
                cfg.RECORD_MAX_SECONDS > 0
                and elapsed_ms >= cfg.RECORD_MAX_SECONDS * 1000
            ):
                break
    finally:
        elapsed_ms = time.ticks_ms() - start_ms
        if recorder is not None:
            try:
                recorder.record_finish()
            except Exception as exc:
                print("video finish failed:", exc)
        log_file.flush()
        log_file.close()
        stats.video_frames = int(elapsed_ms * cfg.RECORD_FPS / 1000)
        try:
            stats.encoded_bytes = os.stat(video_path)[6]
        except Exception:
            stats.encoded_bytes = 0
        with open(summary_path, "w") as output:
            output.write(stats.summary(elapsed_ms))
        try:
            os.sync()
        except Exception:
            pass
        print("recording stopped:", run_dir)
        print(stats.summary(elapsed_ms))


if __name__ == "__main__":
    main()
