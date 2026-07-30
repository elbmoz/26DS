"""Draw synchronized tracking annotations without covering the camera image."""

import cv2
import numpy


STATUS_BAR_HEIGHT = 54
_DEFAULT_CAMERA_SIZE = (640, 480)
_FOOTER_COLOR = (15, 15, 15)


def _scale_point(point, camera_size, frame_size):
    if (
        not point
        or len(point) < 2
        or point[0] is None
        or point[1] is None
        or not camera_size
    ):
        return None
    camera_width, camera_height = camera_size
    frame_width, frame_height = frame_size
    return (
        int(round(float(point[0]) * frame_width / camera_width)),
        int(round(float(point[1]) * frame_height / camera_height)),
    )


def _draw_scene_overlay(frame, tracking, status, synchronized):
    height, width = frame.shape[:2]
    camera_size = _DEFAULT_CAMERA_SIZE
    if status and status.get("camera_size"):
        camera_size = tuple(status["camera_size"])

    roi = None
    axis_source_start = None
    axis_source_end = None
    if tracking and synchronized:
        axis_values = (
            tracking.get("axis_x0"),
            tracking.get("axis_y0"),
            tracking.get("axis_x1"),
            tracking.get("axis_y1"),
        )
        if all(value is not None for value in axis_values):
            axis_source_start = axis_values[:2]
            axis_source_end = axis_values[2:]
        roi_values = (
            tracking.get("roi_x"),
            tracking.get("roi_y"),
            tracking.get("roi_w"),
            tracking.get("roi_h"),
        )
        if all(value is not None for value in roi_values):
            roi = roi_values
    elif not tracking and status:
        roi = status.get("roi")
        axis_source_start = status.get("axis_start")
        axis_source_end = status.get("axis_end")

    if roi and len(roi) == 4:
        start = _scale_point(roi[:2], camera_size, (width, height))
        end = _scale_point(
            (roi[0] + roi[2], roi[1] + roi[3]),
            camera_size,
            (width, height),
        )
        if start and end:
            cv2.rectangle(frame, start, end, (255, 120, 30), 1)

    axis_start = _scale_point(
        axis_source_start, camera_size, (width, height)
    )
    axis_end = _scale_point(
        axis_source_end, camera_size, (width, height)
    )
    if axis_start and axis_end:
        cv2.line(frame, axis_start, axis_end, (0, 220, 0), 2)
        config = (status or {}).get("config") or {}
        target_position = float(config.get("target_position", 0.5))
        target = (
            int(
                round(
                    axis_start[0]
                    + target_position * (axis_end[0] - axis_start[0])
                )
            ),
            int(
                round(
                    axis_start[1]
                    + target_position * (axis_end[1] - axis_start[1])
                )
            ),
        )
        cv2.drawMarker(
            frame,
            target,
            (255, 0, 255),
            cv2.MARKER_CROSS,
            18,
            2,
        )

    if tracking and synchronized:
        for candidate in tracking.get("candidates") or ():
            if not candidate or len(candidate) < 2:
                continue
            candidate_center = _scale_point(
                candidate[:2], camera_size, (width, height)
            )
            if candidate_center:
                cv2.circle(frame, candidate_center, 3, (255, 200, 0), 1)

    if not tracking or not tracking.get("valid") or not synchronized:
        return

    control_center = _scale_point(
        (tracking.get("track_x"), tracking.get("track_y")),
        camera_size,
        (width, height),
    )
    if control_center is None:
        return
    radius = max(
        3,
        int(
            round(
                float(tracking.get("radius") or 3)
                * width
                / camera_size[0]
            )
        ),
    )
    if (
        tracking.get("measured")
        and tracking.get("measurement_x") is not None
        and tracking.get("measurement_y") is not None
    ):
        measured_center = _scale_point(
            (
                tracking.get("measurement_x"),
                tracking.get("measurement_y"),
            ),
            camera_size,
            (width, height),
        )
        if measured_center:
            cv2.circle(frame, measured_center, radius, (0, 0, 255), 2)
            cv2.drawMarker(
                frame,
                measured_center,
                (0, 0, 255),
                cv2.MARKER_CROSS,
                10,
                1,
            )
    control_color = (
        (0, 220, 255) if tracking.get("measured") else (0, 165, 255)
    )
    cv2.drawMarker(
        frame,
        control_center,
        control_color,
        cv2.MARKER_DIAMOND,
        10,
        1,
    )


def _state_style(tracking, synchronized):
    if not tracking:
        return "WAIT", (0, 180, 255)
    if not synchronized:
        return "UNSYNC", (0, 0, 255)
    if tracking.get("measured"):
        return "MEAS", (0, 220, 0)
    if tracking.get("valid"):
        return "PRED", (0, 220, 255)
    return "LOST", (0, 0, 255)


def _put_text_fit(
    image,
    text,
    origin,
    max_width,
    scale,
    color,
    thickness=1,
):
    if max_width <= 0:
        return
    (text_width, _text_height), _baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
    )
    if text_width > max_width:
        scale = max(0.28, scale * max_width / max(1, text_width))
    cv2.putText(
        image,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )


def _draw_status_footer(canvas, video_height, tracking, recording, sync_info):
    width = canvas.shape[1]
    footer_top = int(video_height)
    cv2.line(
        canvas,
        (0, footer_top),
        (width, footer_top),
        (55, 55, 55),
        1,
    )

    synchronized = bool(sync_info.get("matched"))
    state_text, state_color = _state_style(tracking, synchronized)
    state_origin = (8, footer_top + 20)
    cv2.putText(
        canvas,
        state_text,
        state_origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        state_color,
        2,
        cv2.LINE_AA,
    )
    (state_width, _), _ = cv2.getTextSize(
        state_text,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        2,
    )

    recording_width = 0
    if recording:
        recording_width = 48
        cv2.circle(
            canvas,
            (width - 42, footer_top + 15),
            5,
            (0, 0, 255),
            -1,
        )
        cv2.putText(
            canvas,
            "REC",
            (width - 34, footer_top + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

    if tracking:
        error_value = tracking.get("error_px")
        velocity_value = tracking.get("velocity_px_s")
        pipe_text = (
            "OK"
            if tracking.get("pipe_valid")
            else "STALE"
        )
        detail = "fps={:.1f} det={}ms err={}px vel={}px/s pipe={}".format(
            float(tracking.get("fps") or 0),
            tracking.get("detect_ms", ""),
            "--" if error_value is None else error_value,
            (
                "--"
                if velocity_value is None
                else "{:.0f}".format(float(velocity_value))
            ),
            pipe_text,
        )
    else:
        detail = "waiting for synchronized telemetry"
    detail_x = 18 + state_width
    _put_text_fit(
        canvas,
        detail,
        (detail_x, footer_top + 20),
        width - detail_x - recording_width - 6,
        0.40,
        (235, 235, 235),
    )

    latency_ms = sync_info.get("video_latency_ms")
    match_delta_ms = sync_info.get("match_delta_ms")
    sequence = "" if not tracking else tracking.get("seq", "")
    latency_text = "delay={}ms  sync={}ms  skipped={}  seq={}".format(
        "?"
        if latency_ms is None
        else "{:.0f}".format(float(latency_ms)),
        "?"
        if match_delta_ms is None
        else "{:+.0f}".format(float(match_delta_ms)),
        int(sync_info.get("dropped_frames") or 0),
        sequence,
    )
    _put_text_fit(
        canvas,
        latency_text,
        (8, footer_top + 44),
        width - 16,
        0.40,
        (120, 230, 255),
    )


def build_preview_frame(
    frame,
    tracking,
    status,
    recording,
    sync_info=None,
):
    """Return a display frame with a non-occluding status footer.

    The input camera frame is copied directly into the display canvas before
    annotations are drawn.  The footer is appended below the full camera
    image, so every source pixel remains visible and snapshots cannot hide a
    future ROI near the upper edge.
    """
    sync_info = dict(sync_info or {})
    height, width = frame.shape[:2]
    canvas = numpy.empty(
        (height + STATUS_BAR_HEIGHT, width, 3),
        dtype=frame.dtype,
    )
    canvas[:height] = frame
    canvas[height:] = _FOOTER_COLOR
    video_frame = canvas[:height]
    _draw_scene_overlay(
        video_frame,
        tracking,
        status,
        synchronized=bool(sync_info.get("matched")),
    )
    _draw_status_footer(
        canvas,
        height,
        tracking,
        bool(recording),
        sync_info,
    )
    return canvas
