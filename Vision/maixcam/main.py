"""Low-latency steel-ball tracking for the pipe balance device."""

from maix import app, camera, display, err, image, pinmap, time, uart

import ball_config as cfg
from ball_detector import LabBallDetector
from ball_tracker_core import BallTracker
from loop_timing import periodic_due
from pipe_pose import GreenPipePoseDetector, TapeEndpointPipePoseDetector
from stm32_link import Stm32Link


def init_uart():
    if not cfg.UART_ENABLED:
        return None
    try:
        err.check_raise(
            pinmap.set_pin_function(cfg.UART_TX_PIN, "UART1_TX"),
            "failed to map MaixCAM UART1 TX",
        )
        err.check_raise(
            pinmap.set_pin_function(cfg.UART_RX_PIN, "UART1_RX"),
            "failed to map MaixCAM UART1 RX",
        )
        return uart.UART(cfg.UART_DEVICE, cfg.UART_BAUD)
    except Exception as exc:
        print("UART disabled:", exc)
        return None


def init_display():
    if not cfg.DISPLAY_ENABLED:
        return None
    try:
        return display.Display()
    except Exception as exc:
        print("display disabled:", exc)
        return None


def configure_camera(cam):
    cam.skip_frames(cfg.CAMERA_WARMUP_FRAMES)
    if cfg.CAMERA_EXPOSURE_US is not None:
        try:
            cam.exposure(int(cfg.CAMERA_EXPOSURE_US))
            print("manual exposure us:", cfg.CAMERA_EXPOSURE_US)
        except Exception as exc:
            print("manual exposure unavailable:", exc)
    if cfg.CAMERA_GAIN is not None:
        try:
            cam.gain(int(cfg.CAMERA_GAIN))
            print("manual gain:", cfg.CAMERA_GAIN)
        except Exception as exc:
            print("manual gain unavailable:", exc)


def build_detector():
    return LabBallDetector(
        frame_width=cfg.CAMERA_WIDTH,
        frame_height=cfg.CAMERA_HEIGHT,
        full_roi=cfg.ROI,
        thresholds=cfg.BALL_LAB_THRESHOLDS,
        local_width=cfg.LOCAL_SEARCH_WIDTH_PX,
        local_height=cfg.LOCAL_SEARCH_HEIGHT_PX,
        local_fallback_interval_misses=(
            cfg.LOCAL_FALLBACK_INTERVAL_MISSES
        ),
        min_width=cfg.BLOB_MIN_WIDTH,
        max_width=cfg.BLOB_MAX_WIDTH,
        min_height=cfg.BLOB_MIN_HEIGHT,
        max_height=cfg.BLOB_MAX_HEIGHT,
        min_pixels=cfg.BLOB_MIN_PIXELS,
        max_pixels=cfg.BLOB_MAX_PIXELS,
        min_density=cfg.BLOB_MIN_DENSITY,
        max_aspect=cfg.BLOB_MAX_ASPECT,
        merge_blobs=cfg.BLOB_MERGE_BLOBS,
        merge_margin=cfg.BLOB_MERGE_MARGIN,
        blob_x_stride=cfg.BLOB_X_STRIDE,
        blob_y_stride=cfg.BLOB_Y_STRIDE,
        circle_enabled=cfg.CIRCLE_RECOVERY_ENABLED,
        circle_threshold=cfg.CIRCLE_THRESHOLD,
        circle_min_radius=cfg.CIRCLE_MIN_RADIUS,
        circle_max_radius=cfg.CIRCLE_MAX_RADIUS,
        circle_x_stride=cfg.CIRCLE_X_STRIDE,
        circle_y_stride=cfg.CIRCLE_Y_STRIDE,
        circle_radius_step=cfg.CIRCLE_RADIUS_STEP,
        circle_acquire_enabled=cfg.CIRCLE_ACQUIRE_ENABLED,
        circle_acquire_interval_frames=(
            cfg.CIRCLE_ACQUIRE_INTERVAL_FRAMES
        ),
        circle_acquire_roi=cfg.CIRCLE_ACQUIRE_ROI,
        circle_track_interval_frames=cfg.CIRCLE_TRACK_INTERVAL_FRAMES,
        circle_track_endpoint_only=cfg.CIRCLE_TRACK_ENDPOINT_ONLY,
        circle_x_margin=cfg.CIRCLE_X_MARGIN,
        circle_y_margin=cfg.CIRCLE_Y_MARGIN,
        circle_r_margin=cfg.CIRCLE_R_MARGIN,
        circle_color_filter=cfg.CIRCLE_COLOR_FILTER_ENABLED,
        circle_max_chroma=cfg.CIRCLE_MAX_CHROMA,
        circle_dark_value=cfg.CIRCLE_DARK_VALUE,
        circle_min_neutral_samples=cfg.CIRCLE_MIN_NEUTRAL_SAMPLES,
        circle_max_above_center=cfg.CIRCLE_MAX_ABOVE_ROI_CENTER_PX,
        circle_max_below_center=cfg.CIRCLE_MAX_BELOW_ROI_CENTER_PX,
        blob_center_bias_along_axis_px=(
            cfg.BLOB_CENTER_BIAS_ALONG_AXIS_PX
        ),
        blob_center_bias_min_quality=(
            cfg.BLOB_CENTER_BIAS_MIN_QUALITY
        ),
        circle_trigger_min_quality=cfg.CIRCLE_TRIGGER_MIN_QUALITY,
        circle_trigger_max_axis_distance_px=(
            cfg.CIRCLE_TRIGGER_MAX_AXIS_DISTANCE_PX
        ),
        circle_endpoint_position=cfg.CIRCLE_ENDPOINT_POSITION,
        circle_endpoint_inward_bias_px=(
            cfg.CIRCLE_ENDPOINT_INWARD_BIAS_PX
        ),
        circle_left_endpoint_inward_bias_px=(
            cfg.CIRCLE_LEFT_ENDPOINT_INWARD_BIAS_PX
        ),
        circle_right_endpoint_inward_bias_px=(
            cfg.CIRCLE_RIGHT_ENDPOINT_INWARD_BIAS_PX
        ),
    )


def build_tracker():
    return BallTracker(
        axis_start=cfg.AXIS_START,
        axis_end=cfg.AXIS_END,
        target_position=cfg.TARGET_POSITION,
        max_axis_distance_px=cfg.MAX_AXIS_DISTANCE_PX,
        max_below_axis_distance_px=cfg.MAX_BELOW_AXIS_DISTANCE_PX,
        max_frame_jump_px=cfg.MAX_FRAME_JUMP_PX,
        acquire_position_margin=cfg.ACQUIRE_POSITION_MARGIN,
        track_position_margin=cfg.TRACK_POSITION_MARGIN,
        acquire_endpoint_inset=cfg.ACQUIRE_ENDPOINT_INSET,
        track_endpoint_inset=cfg.TRACK_ENDPOINT_INSET,
        acquire_min_quality=cfg.ACQUIRE_MIN_QUALITY,
        track_min_quality=cfg.TRACK_MIN_QUALITY,
        smoothing_alpha=cfg.POSITION_ALPHA,
        velocity_beta=cfg.VELOCITY_BETA,
        lateral_alpha=cfg.LATERAL_ALPHA,
        confirm_frames=cfg.CONFIRM_FRAMES,
        coast_frames=cfg.COAST_FRAMES,
        endpoint_coast_frames=cfg.ENDPOINT_COAST_FRAMES,
        memory_frames=cfg.TRACK_MEMORY_FRAMES,
        fixture_exclusions=cfg.FIXTURE_EXCLUSION_ZONES,
        fixture_blob_override_quality=(
            cfg.FIXTURE_BLOB_OVERRIDE_QUALITY
        ),
        fixture_soft_radius_scale=cfg.FIXTURE_SOFT_RADIUS_SCALE,
        fixture_soft_penalty_per_px=(
            cfg.FIXTURE_SOFT_PENALTY_PER_PX
        ),
        endpoint_snap_left_position=(
            cfg.ENDPOINT_SNAP_LEFT_POSITION
        ),
        endpoint_snap_right_position=(
            cfg.ENDPOINT_SNAP_RIGHT_POSITION
        ),
        endpoint_snap_enter=cfg.ENDPOINT_SNAP_ENTER,
        endpoint_snap_exit=cfg.ENDPOINT_SNAP_EXIT,
        endpoint_snap_confirm_frames=(
            cfg.ENDPOINT_SNAP_CONFIRM_FRAMES
        ),
    )


def build_pipe_detector():
    if not cfg.PIPE_POSE_ENABLED:
        return None
    if cfg.PIPE_POSE_MODE == "right_tape":
        return TapeEndpointPipePoseDetector(
            frame_width=cfg.CAMERA_WIDTH,
            frame_height=cfg.CAMERA_HEIGHT,
            right_search_roi=cfg.PIPE_TAPE_RIGHT_SEARCH_ROI,
            fallback_roi=cfg.ROI,
            fixed_left_endpoint=cfg.PIPE_TAPE_LEFT_ENDPOINT,
            fallback_right_endpoint=cfg.PIPE_TAPE_RIGHT_ENDPOINT,
            thresholds=cfg.PIPE_TAPE_LAB_THRESHOLDS,
            detect_interval_frames=cfg.PIPE_TAPE_DETECT_INTERVAL_FRAMES,
            min_width_px=cfg.PIPE_TAPE_MIN_WIDTH_PX,
            max_width_px=cfg.PIPE_TAPE_MAX_WIDTH_PX,
            min_height_px=cfg.PIPE_TAPE_MIN_HEIGHT_PX,
            max_height_px=cfg.PIPE_TAPE_MAX_HEIGHT_PX,
            min_pixels=cfg.PIPE_TAPE_MIN_PIXELS,
            x_stride=cfg.PIPE_TAPE_X_STRIDE,
            y_stride=cfg.PIPE_TAPE_Y_STRIDE,
            expected_right_x=cfg.PIPE_TAPE_EXPECTED_RIGHT_X,
            max_right_x_distance_px=(
                cfg.PIPE_TAPE_MAX_RIGHT_X_DISTANCE_PX
            ),
            fixed_right_x=cfg.PIPE_TAPE_FIXED_RIGHT_X,
            max_right_y_step_px=cfg.PIPE_TAPE_MAX_RIGHT_Y_STEP_PX,
            min_axis_length_px=cfg.PIPE_TAPE_MIN_AXIS_LENGTH_PX,
            max_axis_length_px=cfg.PIPE_TAPE_MAX_AXIS_LENGTH_PX,
            max_abs_angle_deg=cfg.PIPE_MAX_ABS_ANGLE_DEG,
            endpoint_from_blob_right_edge=(
                cfg.PIPE_TAPE_ENDPOINT_FROM_BLOB_RIGHT_EDGE
            ),
            endpoint_x_offset_px=cfg.PIPE_TAPE_ENDPOINT_X_OFFSET_PX,
            smoothing_alpha=cfg.PIPE_SMOOTHING_ALPHA,
            roi_along_margin_px=cfg.PIPE_ROI_ALONG_MARGIN_PX,
            roi_lateral_margin_px=cfg.PIPE_ROI_LATERAL_MARGIN_PX,
            roi_start_margin_px=cfg.PIPE_ROI_START_MARGIN_PX,
            max_stale_frames=cfg.PIPE_MAX_STALE_FRAMES,
        )
    return GreenPipePoseDetector(
        frame_width=cfg.CAMERA_WIDTH,
        frame_height=cfg.CAMERA_HEIGHT,
        search_roi=cfg.PIPE_SEARCH_ROI,
        fallback_roi=cfg.ROI,
        fallback_axis_start=cfg.AXIS_START,
        fallback_axis_end=cfg.AXIS_END,
        thresholds=cfg.PIPE_LAB_THRESHOLDS,
        detect_interval_frames=cfg.PIPE_DETECT_INTERVAL_FRAMES,
        min_length_px=cfg.PIPE_MIN_LENGTH_PX,
        max_length_px=cfg.PIPE_MAX_LENGTH_PX,
        min_width_px=cfg.PIPE_MIN_WIDTH_PX,
        max_width_px=cfg.PIPE_MAX_WIDTH_PX,
        min_aspect=cfg.PIPE_MIN_ASPECT,
        min_pixels=cfg.PIPE_MIN_PIXELS,
        merge_blobs=cfg.PIPE_MERGE_BLOBS,
        merge_margin=cfg.PIPE_MERGE_MARGIN,
        x_stride=cfg.PIPE_X_STRIDE,
        y_stride=cfg.PIPE_Y_STRIDE,
        expected_center=cfg.PIPE_EXPECTED_CENTER,
        max_center_distance_px=cfg.PIPE_MAX_CENTER_DISTANCE_PX,
        max_abs_angle_deg=cfg.PIPE_MAX_ABS_ANGLE_DEG,
        fixed_axis_center=cfg.PIPE_FIXED_AXIS_CENTER,
        fixed_axis_length_px=cfg.PIPE_FIXED_AXIS_LENGTH_PX,
        pose_search_along_margin_px=(
            cfg.PIPE_POSE_SEARCH_ALONG_MARGIN_PX
        ),
        pose_search_lateral_margin_px=(
            cfg.PIPE_POSE_SEARCH_LATERAL_MARGIN_PX
        ),
        smoothing_alpha=cfg.PIPE_SMOOTHING_ALPHA,
        axis_inset_px=cfg.PIPE_AXIS_INSET_PX,
        roi_along_margin_px=cfg.PIPE_ROI_ALONG_MARGIN_PX,
        roi_lateral_margin_px=cfg.PIPE_ROI_LATERAL_MARGIN_PX,
        max_stale_frames=cfg.PIPE_MAX_STALE_FRAMES,
        broad_retry_interval_updates=(
            cfg.PIPE_BROAD_RETRY_INTERVAL_UPDATES
        ),
        fixed_search_roi=cfg.PIPE_FIXED_SEARCH_ROI,
    )


def process_frame(img, now_ms, frame_id, detector, tracker, pipe_detector):
    """Update pipe geometry, detect the ball and advance its tracker."""
    if pipe_detector is None:
        pipe_state = {
            "axis_start": tuple(cfg.AXIS_START),
            "axis_end": tuple(cfg.AXIS_END),
            "ball_roi": tuple(cfg.ROI),
            "measured": False,
            "valid": False,
            "age_frames": 0,
            "raw_blob_count": 0,
            "score": 0.0,
            "length": 0.0,
            "width": 0.0,
        }
    else:
        pipe_state = pipe_detector.update(img, frame_id)

    tracker.set_axis(pipe_state["axis_start"], pipe_state["axis_end"])
    detector.set_full_roi(pipe_state["ball_roi"])
    detector.set_axis(pipe_state["axis_start"], pipe_state["axis_end"])
    # After coasting expires, the remembered point is useful only as history.
    # Searching locally around it can hide a fast ball at the opposite end.
    predicted = (
        tracker.predicted_point(now_ms)
        if tracker.confirmed
        else None
    )
    predicted_x = None if predicted is None else predicted[0]
    predicted_y = None if predicted is None else predicted[1]
    detection = detector.detect(
        img,
        predicted_x=predicted_x,
        predicted_y=predicted_y,
    )
    detection["full_roi"] = tuple(detector.full_roi)
    detection["axis_start"] = tuple(tracker.axis_start)
    detection["axis_end"] = tuple(tracker.axis_end)
    detection["pipe"] = pipe_state
    state = tracker.update(detection["candidates"], now_ms)
    if cfg.REQUIRE_VALID_PIPE_POSE and not pipe_state["valid"]:
        # Never send a geometrically referenced control error after the pipe
        # endpoint model has gone stale.  The tracker may keep its internal
        # memory for fast recovery, but the STM32 sees an explicit invalid
        # sample until a fresh pipe pose is available again.
        state = dict(state)
        state["valid"] = False
        state["measured"] = False
        state["coasting"] = False
    return detection, state


def draw_overlay(img, detection, state, fps_value, measured_ratio):
    roi_x, roi_y, roi_w, roi_h = detection.get(
        "full_roi", tuple(cfg.ROI)
    )
    axis_x0, axis_y0 = detection.get(
        "axis_start", tuple(cfg.AXIS_START)
    )
    axis_x1, axis_y1 = detection.get(
        "axis_end", tuple(cfg.AXIS_END)
    )
    axis_x0 = int(round(axis_x0))
    axis_y0 = int(round(axis_y0))
    axis_x1 = int(round(axis_x1))
    axis_y1 = int(round(axis_y1))
    target_x = int(
        round(axis_x0 + cfg.TARGET_POSITION * (axis_x1 - axis_x0))
    )
    target_y = int(
        round(axis_y0 + cfg.TARGET_POSITION * (axis_y1 - axis_y0))
    )

    img.draw_rect(roi_x, roi_y, roi_w, roi_h, image.COLOR_BLUE, 2)
    search_x, search_y, search_w, search_h = detection["search_roi"]
    if detection["search_roi"] != tuple(detection["full_roi"]):
        img.draw_rect(
            search_x,
            search_y,
            search_w,
            search_h,
            image.COLOR_YELLOW,
            1,
        )
    img.draw_line(
        axis_x0, axis_y0, axis_x1, axis_y1, image.COLOR_GREEN, 2
    )
    img.draw_cross(target_x, target_y, image.COLOR_RED, 12, 2)

    for blob in detection["blobs"]:
        img.draw_rect(
            blob.x(), blob.y(), blob.w(), blob.h(), image.COLOR_BLUE, 1
        )

    if state["valid"]:
        x = int(round(state["x"]))
        y = int(round(state["y"]))
        radius = max(3, int(round(state["radius"])))
        color = image.COLOR_RED if state["measured"] else image.COLOR_YELLOW
        img.draw_circle(x, y, radius, color, 3)
        img.draw_cross(x, y, color, 8, 2)
        mode = "MEAS" if state["measured"] else "PRED"
        status = "{} e={} v={:.0f} pipe={}".format(
            mode,
            state["error_px"],
            state["velocity_px_s"],
            "OK" if detection["pipe"]["valid"] else "STALE",
        )
        status_color = image.COLOR_GREEN
    else:
        status = "LOST raw={} cand={}".format(
            detection["raw_count"], len(detection["candidates"])
        )
        status_color = image.COLOR_RED

    img.draw_string(8, 8, status, status_color, 1.0)
    img.draw_string(
        8,
        31,
        "FPS {:.1f} hit {:.0f}%".format(fps_value, measured_ratio * 100.0),
        image.COLOR_WHITE,
        0.9,
    )


def main():
    stm32_link = Stm32Link(
        init_uart(),
        output_scale=cfg.CONTROL_OUTPUT_SCALE,
    )
    screen = init_display()
    cam = camera.Camera(
        width=cfg.CAMERA_WIDTH,
        height=cfg.CAMERA_HEIGHT,
        fps=cfg.CAMERA_FPS,
        buff_num=cfg.CAMERA_BUFFER_COUNT,
    )
    configure_camera(cam)

    detector = build_detector()
    tracker = build_tracker()
    pipe_detector = build_pipe_detector()

    send_period_ms = max(1, int(1000 / cfg.TELEMETRY_HZ))
    loop_period_ms = (
        max(1, int(1000 / cfg.CONTROL_LOOP_HZ))
        if cfg.CONTROL_LOOP_HZ > 0
        else None
    )
    console_period_ms = max(1, int(1000 / cfg.CONSOLE_HZ))
    preview_period_ms = max(1, int(1000 / cfg.PREVIEW_HZ))
    next_send_ms = time.ticks_ms()
    next_loop_ms = next_send_ms
    last_console_ms = 0
    last_preview_ms = 0

    window_frames = 0
    window_measured = 0
    window_valid = 0
    window_coast = 0
    window_detect_ms = 0
    measured_ratio = 0.0
    frame_id = 0

    while not app.need_exit():
        stm32_link.poll_commands()
        img = cam.read()
        now_ms = time.ticks_ms()
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

        window_frames += 1
        window_measured += 1 if state["measured"] else 0
        window_valid += 1 if state["valid"] else 0
        window_coast += 1 if state["coasting"] else 0
        window_detect_ms += detect_ms

        send_due, next_send_ms = periodic_due(
            now_ms, next_send_ms, send_period_ms
        )
        if send_due:
            stm32_link.send_state(state)

        if now_ms - last_console_ms >= console_period_ms:
            measured_ratio = (
                float(window_measured) / window_frames
                if window_frames
                else 0.0
            )
            valid_ratio = (
                float(window_valid) / window_frames
                if window_frames
                else 0.0
            )
            average_detect_ms = (
                float(window_detect_ms) / window_frames
                if window_frames
                else 0.0
            )
            print(
                "fps={:.1f} measured={:.1f}% valid={:.1f}% "
                "coast={} raw={} cand={} detect_ms={:.2f} "
                "error={} velocity={:.0f}".format(
                    fps_value,
                    measured_ratio * 100.0,
                    valid_ratio * 100.0,
                    window_coast,
                    detection["raw_count"],
                    len(detection["candidates"]),
                    average_detect_ms,
                    state["error_px"],
                    state["velocity_px_s"],
                )
            )
            window_frames = 0
            window_measured = 0
            window_valid = 0
            window_coast = 0
            window_detect_ms = 0
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


if __name__ == "__main__":
    if cfg.APP_MODE == "stream":
        from stream_tracking import main as stream_tracking_main

        stream_tracking_main()
    elif cfg.APP_MODE == "record":
        from record_tracking import main as record_tracking_main

        record_tracking_main()
    elif cfg.APP_MODE == "serve":
        from serve_results import main as serve_results_main

        serve_results_main()
    else:
        main()
