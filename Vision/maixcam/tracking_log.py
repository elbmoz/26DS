"""CSV formatting and run statistics for synchronized tracking tests."""


CSV_FIELDS = (
    "frame_id",
    "video_frame_id",
    "elapsed_ms",
    "loop_dt_ms",
    "fps",
    "encoded_bytes",
    "measured",
    "valid",
    "coasting",
    "track_x",
    "track_y",
    "measurement_x",
    "measurement_y",
    "measurement_radius",
    "radius",
    "position",
    "position_px",
    "travel_position_px",
    "travel_length_px",
    "target_axis_px",
    "error_px",
    "lateral_px",
    "velocity_px_s",
    "quality",
    "position_rejects",
    "lateral_rejects",
    "fixture_rejects",
    "quality_rejects",
    "jump_rejects",
    "hits",
    "misses",
    "raw_blob_count",
    "candidate_count",
    "local_search",
    "fell_back",
    "axis_x0",
    "axis_y0",
    "axis_x1",
    "axis_y1",
    "roi_x",
    "roi_y",
    "roi_w",
    "roi_h",
    "pipe_measured",
    "pipe_valid",
    "pipe_age_frames",
    "pipe_blob_count",
    "pipe_length",
    "pipe_width",
    "pipe_score",
    "detect_ms",
)


def _number(value, digits=3):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    return ("{:.%df}" % digits).format(float(value))


def csv_header():
    return ",".join(CSV_FIELDS) + "\n"


def tracking_row(
    frame_id,
    video_frame_id,
    elapsed_ms,
    loop_dt_ms,
    fps_value,
    encoded_bytes,
    state,
    detection,
    detect_ms,
    full_roi,
):
    axis_start = detection.get("axis_start", (None, None))
    axis_end = detection.get("axis_end", (None, None))
    pipe_state = detection.get("pipe", {})
    roi_x, roi_y, roi_w, roi_h = full_roi
    values = (
        int(frame_id),
        video_frame_id,
        int(elapsed_ms),
        int(loop_dt_ms),
        fps_value,
        int(encoded_bytes),
        bool(state["measured"]),
        bool(state["valid"]),
        bool(state["coasting"]),
        state["x"] if state["valid"] else None,
        state["y"] if state["valid"] else None,
        state.get("measurement_x"),
        state.get("measurement_y"),
        state.get("measurement_radius"),
        state["radius"] if state["valid"] else None,
        state["position"] if state["valid"] else None,
        state["position_px"] if state["valid"] else None,
        state.get("travel_position_px") if state["valid"] else None,
        state.get("travel_length_px") if state["valid"] else None,
        state.get("target_axis_px") if state["valid"] else None,
        state["error_px"] if state["valid"] else None,
        state["lateral_px"] if state["valid"] else None,
        state["velocity_px_s"] if state["valid"] else None,
        state["quality"],
        state.get("position_rejects", 0),
        state.get("lateral_rejects", 0),
        state.get("fixture_rejects", 0),
        state.get("quality_rejects", 0),
        state.get("jump_rejects", 0),
        state["hits"],
        state["misses"],
        detection["raw_count"],
        len(detection["candidates"]),
        tuple(detection["search_roi"]) != tuple(full_roi),
        bool(detection["fell_back"]),
        axis_start[0],
        axis_start[1],
        axis_end[0],
        axis_end[1],
        roi_x,
        roi_y,
        roi_w,
        roi_h,
        pipe_state.get("measured", False),
        pipe_state.get("valid", False),
        pipe_state.get("age_frames", 0),
        pipe_state.get("raw_blob_count", 0),
        pipe_state.get("length"),
        pipe_state.get("width"),
        pipe_state.get("score"),
        int(detect_ms),
    )
    return ",".join(_number(value) for value in values) + "\n"


class RunStats:
    def __init__(self):
        self.frames = 0
        self.measured = 0
        self.valid = 0
        self.coasting = 0
        self.detect_ms = 0
        self.encoded_bytes = 0
        self.video_frames = 0

    def update(self, state, detect_ms, encoded_bytes):
        self.frames += 1
        self.measured += 1 if state["measured"] else 0
        self.valid += 1 if state["valid"] else 0
        self.coasting += 1 if state["coasting"] else 0
        self.detect_ms += int(detect_ms)
        self.encoded_bytes += int(encoded_bytes)
        self.video_frames += 1 if encoded_bytes > 0 else 0

    def summary(self, elapsed_ms):
        seconds = max(0.001, float(elapsed_ms) / 1000.0)
        frames = max(1, self.frames)
        return (
            "frames={}\n"
            "duration_s={:.3f}\n"
            "effective_fps={:.3f}\n"
            "measured_ratio={:.6f}\n"
            "valid_ratio={:.6f}\n"
            "coast_ratio={:.6f}\n"
            "average_detect_ms={:.3f}\n"
            "nominal_video_frames={}\n"
            "requested_video_fps={:.3f}\n"
            "video_bytes={}\n"
        ).format(
            self.frames,
            seconds,
            self.frames / seconds,
            float(self.measured) / frames,
            float(self.valid) / frames,
            float(self.coasting) / frames,
            float(self.detect_ms) / frames,
            self.video_frames,
            self.video_frames / seconds,
            self.encoded_bytes,
        )
