"""Versioned, hardware-independent protocol for live vision telemetry.

The module intentionally has no ``maix`` imports so the exact same packet
validation code can be unit-tested and reused by the Windows receiver.
"""

import json
import math


PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 4096

# Only vision parameters that are safe to change while tracking are exposed.
# Camera geometry and LAB thresholds remain file-based calibration values.
TUNABLE_SPECS = {
    "model": ("model_path", None, None),
    "confidence": ("float", 0.01, 0.99),
    "valid_confidence": ("float", 0.01, 0.99),
    "iou": ("float", 0.01, 0.99),
    "target_position": ("float", 0.05, 0.95),
    "travel_start_px": ("float", 0.0, 120.0),
    "travel_end_px": ("float", 0.0, 120.0),
    "position_alpha": ("float", 0.05, 1.00),
    "velocity_beta": ("float", 0.00, 1.00),
    "lateral_alpha": ("float", 0.05, 1.00),
    "max_axis_distance_px": ("float", 5.0, 80.0),
    "max_below_axis_distance_px": ("float", 3.0, 80.0),
    "max_frame_jump_px": ("float", 10.0, 240.0),
    "acquire_position_margin": ("float", 0.00, 0.10),
    "track_position_margin": ("float", 0.00, 0.08),
    "acquire_endpoint_inset": ("float", 0.00, 0.12),
    "track_endpoint_inset": ("float", 0.00, 0.08),
    "acquire_min_quality": ("float", 0.0, 200.0),
    "track_min_quality": ("float", 0.0, 200.0),
    "coast_frames": ("int", 0, 15),
    "local_search_width_px": ("int", 40, 470),
    "circle_threshold": ("int", 100, 5000),
    "circle_min_radius": ("int", 6, 24),
    "circle_max_radius": ("int", 8, 32),
}


class ProtocolError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = str(code)


def _rounded(value, digits=3):
    if value is None:
        return None
    return round(float(value), digits)


def encode_packet(packet):
    payload = json.dumps(packet, separators=(",", ":"), ensure_ascii=True)
    encoded = payload.encode("utf-8")
    if len(encoded) > MAX_PACKET_BYTES:
        raise ProtocolError("packet_too_large", "packet exceeds size limit")
    return encoded


def decode_packet(data):
    if not isinstance(data, (bytes, bytearray)):
        raise ProtocolError("invalid_encoding", "packet must be bytes")
    if len(data) > MAX_PACKET_BYTES:
        raise ProtocolError("packet_too_large", "packet exceeds size limit")
    try:
        packet = json.loads(bytes(data).decode("utf-8"))
    except Exception as exc:
        raise ProtocolError("invalid_json", str(exc))
    if not isinstance(packet, dict):
        raise ProtocolError("invalid_packet", "packet root must be an object")
    if packet.get("v") != PROTOCOL_VERSION:
        raise ProtocolError(
            "unsupported_version",
            "expected protocol version {}".format(PROTOCOL_VERSION),
        )
    return packet


def validate_parameters(params):
    if not isinstance(params, dict):
        return {}, {"params": "must be an object"}

    clean = {}
    errors = {}
    for name, value in params.items():
        spec = TUNABLE_SPECS.get(name)
        if spec is None:
            errors[name] = "unknown parameter"
            continue
        kind, minimum, maximum = spec
        if kind == "model_path":
            if not isinstance(value, str):
                errors[name] = "must be a string"
                continue
            clean_path = value.strip()
            if (
                not clean_path.startswith("/root/models/maixhub/")
                or not clean_path.endswith(".mud")
                or ".." in clean_path.split("/")
                or len(clean_path) > 240
            ):
                errors[name] = (
                    "must be a .mud file under /root/models/maixhub"
                )
                continue
            clean[name] = clean_path
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors[name] = "must be a number"
            continue
        numeric = float(value)
        if not math.isfinite(numeric):
            errors[name] = "must be finite"
            continue

        if numeric < minimum or numeric > maximum:
            errors[name] = "must be in [{}, {}]".format(minimum, maximum)
            continue
        if kind == "int":
            if numeric != int(numeric):
                errors[name] = "must be an integer"
                continue
            clean[name] = int(numeric)
        else:
            clean[name] = numeric
    return clean, errors


def config_snapshot(detector, tracker):
    if tracker is None and detector.__class__.__name__ == "AIBallDetector":
        return {
            "algorithm": "ai",
            "model": detector.config.model_path,
            "target_position": _rounded(
                detector.config.target_position
            ),
            "travel_start_px": _rounded(
                detector.config.travel_start_px
            ),
            "travel_end_px": _rounded(
                detector.config.travel_end_px
            ),
            "confidence": _rounded(detector.config.confidence),
            "valid_confidence": _rounded(
                detector.config.valid_confidence
            ),
            "iou": _rounded(detector.config.iou),
            "coast_frames": int(detector.config.coast_frames),
            "input_size": [
                int(detector.input_width),
                int(detector.input_height),
            ],
        }
    if tracker is None and hasattr(detector, "config"):
        return {
            "algorithm": "v2",
            "target_position": _rounded(
                detector.config.target_position
            ),
            "ball_diameter_px": _rounded(
                detector.config.ball_diameter_px
            ),
        }
    return {
        "target_position": _rounded(tracker.target_position),
        "position_alpha": _rounded(tracker.position_alpha),
        "velocity_beta": _rounded(tracker.velocity_beta),
        "lateral_alpha": _rounded(tracker.lateral_alpha),
        "max_axis_distance_px": _rounded(
            tracker.max_axis_distance_px
        ),
        "max_below_axis_distance_px": _rounded(
            tracker.max_below_axis_distance_px
        ),
        "max_frame_jump_px": _rounded(tracker.max_frame_jump_px),
        "acquire_position_margin": _rounded(
            tracker.acquire_position_margin
        ),
        "track_position_margin": _rounded(
            tracker.track_position_margin
        ),
        "acquire_endpoint_inset": _rounded(
            tracker.acquire_endpoint_inset
        ),
        "track_endpoint_inset": _rounded(
            tracker.track_endpoint_inset
        ),
        "acquire_min_quality": _rounded(tracker.acquire_min_quality),
        "track_min_quality": _rounded(tracker.track_min_quality),
        "coast_frames": int(tracker.coast_frames),
        "local_search_width_px": int(detector.local_width),
        "circle_threshold": int(detector.circle_threshold),
        "circle_min_radius": int(detector.circle_min_radius),
        "circle_max_radius": int(detector.circle_max_radius),
    }


def apply_parameters(clean, detector, tracker, config_module):
    """Apply already validated parameters and return the resulting snapshot."""
    if tracker is None and detector.__class__.__name__ == "AIBallDetector":
        supported = {
            "model",
            "target_position",
            "travel_start_px",
            "travel_end_px",
            "confidence",
            "valid_confidence",
            "iou",
            "coast_frames",
        }
        unsupported = [name for name in clean if name not in supported]
        if unsupported:
            raise ProtocolError(
                "unsupported_parameter",
                "AI rejected unsupported parameters {}".format(
                    ",".join(sorted(unsupported))
                ),
            )
        detector.apply_runtime_config(clean)
        config_module.AI_MODEL_PATH = detector.config.model_path
        config_module.TARGET_POSITION = detector.config.target_position
        config_module.AI_TRAVEL_START_PX = (
            detector.config.travel_start_px
        )
        config_module.AI_TRAVEL_END_PX = detector.config.travel_end_px
        config_module.AI_CONFIDENCE = detector.config.confidence
        config_module.AI_VALID_CONFIDENCE = (
            detector.config.valid_confidence
        )
        config_module.AI_IOU = detector.config.iou
        config_module.AI_COAST_FRAMES = detector.config.coast_frames
        return config_snapshot(detector, tracker)

    if tracker is None and hasattr(detector, "config"):
        unsupported = [
            name for name in clean if name != "target_position"
        ]
        if unsupported:
            raise ProtocolError(
                "unsupported_parameter",
                "V2 only exposes target_position; rejected {}".format(
                    ",".join(sorted(unsupported))
                ),
            )
        if "target_position" in clean:
            value = clean["target_position"]
            detector.config.target_position = value
            detector.tracker.target = value
            config_module.TARGET_POSITION = value
            config_module.V2_TARGET_POSITION = value
        return config_snapshot(detector, tracker)

    if "target_position" in clean:
        value = clean["target_position"]
        tracker.target_position = value
        config_module.TARGET_POSITION = value
    if "position_alpha" in clean:
        value = clean["position_alpha"]
        tracker.position_alpha = value
        config_module.POSITION_ALPHA = value
    if "velocity_beta" in clean:
        value = clean["velocity_beta"]
        tracker.velocity_beta = value
        config_module.VELOCITY_BETA = value
    if "lateral_alpha" in clean:
        value = clean["lateral_alpha"]
        tracker.lateral_alpha = value
        config_module.LATERAL_ALPHA = value
    if "max_axis_distance_px" in clean:
        value = clean["max_axis_distance_px"]
        tracker.max_axis_distance_px = value
        config_module.MAX_AXIS_DISTANCE_PX = value
    if "max_below_axis_distance_px" in clean:
        value = clean["max_below_axis_distance_px"]
        tracker.max_below_axis_distance_px = value
        config_module.MAX_BELOW_AXIS_DISTANCE_PX = value
    if "max_frame_jump_px" in clean:
        value = clean["max_frame_jump_px"]
        tracker.max_frame_jump_px = value
        config_module.MAX_FRAME_JUMP_PX = value
    if "acquire_position_margin" in clean:
        value = clean["acquire_position_margin"]
        tracker.acquire_position_margin = value
        config_module.ACQUIRE_POSITION_MARGIN = value
    if "track_position_margin" in clean:
        value = clean["track_position_margin"]
        tracker.track_position_margin = value
        config_module.TRACK_POSITION_MARGIN = value
    if "acquire_endpoint_inset" in clean:
        value = clean["acquire_endpoint_inset"]
        tracker.acquire_endpoint_inset = value
        config_module.ACQUIRE_ENDPOINT_INSET = value
    if "track_endpoint_inset" in clean:
        value = clean["track_endpoint_inset"]
        tracker.track_endpoint_inset = value
        config_module.TRACK_ENDPOINT_INSET = value
    if "acquire_min_quality" in clean:
        value = clean["acquire_min_quality"]
        tracker.acquire_min_quality = value
        config_module.ACQUIRE_MIN_QUALITY = value
    if "track_min_quality" in clean:
        value = clean["track_min_quality"]
        tracker.track_min_quality = value
        config_module.TRACK_MIN_QUALITY = value
    if "coast_frames" in clean:
        value = clean["coast_frames"]
        tracker.coast_frames = value
        tracker.memory_frames = max(value, tracker.memory_frames)
        config_module.COAST_FRAMES = value
    if "local_search_width_px" in clean:
        value = clean["local_search_width_px"]
        detector.local_width = value
        config_module.LOCAL_SEARCH_WIDTH_PX = value
    if "circle_threshold" in clean:
        value = clean["circle_threshold"]
        detector.circle_threshold = value
        config_module.CIRCLE_THRESHOLD = value
    if "circle_min_radius" in clean:
        value = clean["circle_min_radius"]
        detector.circle_min_radius = value
        config_module.CIRCLE_MIN_RADIUS = value
    if "circle_max_radius" in clean:
        value = clean["circle_max_radius"]
        detector.circle_max_radius = value
        config_module.CIRCLE_MAX_RADIUS = value
    return config_snapshot(detector, tracker)


def make_tracking_packet(
    session_id,
    sequence,
    device_ms,
    frame_id,
    loop_dt_ms,
    fps_value,
    detect_ms,
    state,
    detection,
    q9=None,
):
    valid = bool(state["valid"])
    axis_start = detection.get("axis_start", (None, None))
    axis_end = detection.get("axis_end", (None, None))
    full_roi = detection.get("full_roi", detection["search_roi"])
    pipe_state = detection.get("pipe", {})
    packet = {
        "v": PROTOCOL_VERSION,
        "type": "tracking",
        "session": str(session_id),
        "seq": int(sequence),
        "device_ms": int(device_ms),
        "frame_id": int(frame_id),
        "loop_dt_ms": int(loop_dt_ms),
        "fps": _rounded(fps_value, 2),
        "detect_ms": int(detect_ms),
        "measured": bool(state["measured"]),
        "valid": valid,
        "coasting": bool(state["coasting"]),
        "track_x": _rounded(state["x"]) if valid else None,
        "track_y": _rounded(state["y"]) if valid else None,
        "measurement_x": _rounded(state.get("measurement_x")),
        "measurement_y": _rounded(state.get("measurement_y")),
        "measurement_radius": _rounded(state.get("measurement_radius")),
        "radius": _rounded(state["radius"]) if valid else None,
        "position": _rounded(state["position"], 5) if valid else None,
        "position_px": _rounded(state["position_px"]) if valid else None,
        "travel_position_px": (
            _rounded(state.get("travel_position_px")) if valid else None
        ),
        "travel_length_px": (
            _rounded(state.get("travel_length_px")) if valid else None
        ),
        "target_axis_px": (
            _rounded(state.get("target_axis_px")) if valid else None
        ),
        "error_px": int(state["error_px"]) if valid else None,
        "lateral_px": int(state["lateral_px"]) if valid else None,
        "velocity_px_s": (
            _rounded(state["velocity_px_s"]) if valid else None
        ),
        "quality": _rounded(state["quality"]),
        "position_rejects": int(state.get("position_rejects", 0)),
        "lateral_rejects": int(state.get("lateral_rejects", 0)),
        "fixture_rejects": int(state.get("fixture_rejects", 0)),
        "quality_rejects": int(state.get("quality_rejects", 0)),
        "jump_rejects": int(state.get("jump_rejects", 0)),
        "hits": int(state["hits"]),
        "misses": int(state["misses"]),
        "raw_blob_count": int(detection["raw_count"]),
        "circle_count": int(detection.get("circle_count", 0)),
        "candidate_count": len(detection["candidates"]),
        "candidates": [
            [_rounded(value) for value in candidate[:4]]
            for candidate in detection["candidates"][:8]
        ],
        "algorithm": str(detection.get("algorithm", "")),
        "ai_boxes": [
            [_rounded(value) for value in box[:6]]
            for box in detection.get("boxes", ())[:8]
        ],
        "local_search": bool(
            detection.get(
                "used_local",
                tuple(detection["search_roi"])
                != tuple(
                    detection.get("full_roi", detection["search_roi"])
                ),
            )
        ),
        "fell_back": bool(detection["fell_back"]),
        "axis_x0": _rounded(axis_start[0]),
        "axis_y0": _rounded(axis_start[1]),
        "axis_x1": _rounded(axis_end[0]),
        "axis_y1": _rounded(axis_end[1]),
        "roi_x": int(full_roi[0]),
        "roi_y": int(full_roi[1]),
        "roi_w": int(full_roi[2]),
        "roi_h": int(full_roi[3]),
        "roi_quad": [
            [_rounded(point[0]), _rounded(point[1])]
            for point in (detection.get("roi_quad") or ())
        ],
        "pipe_measured": bool(pipe_state.get("measured", False)),
        "pipe_valid": bool(pipe_state.get("valid", False)),
        "pipe_age_frames": int(pipe_state.get("age_frames", 0)),
        "pipe_blob_count": int(pipe_state.get("raw_blob_count", 0)),
        "pipe_length": _rounded(pipe_state.get("length")),
        "pipe_width": _rounded(pipe_state.get("width")),
        "pipe_score": _rounded(pipe_state.get("score")),
    }
    if q9:
        q9_fields = (
            "seq",
            "seq_gap",
            "mcu_ms",
            "motor_position",
            "angle_x_x10",
            "angle_y_x10",
            "angle_z_x10",
            "angle_x_deg",
            "angle_y_deg",
            "angle_z_deg",
            "imu_valid",
            "position_valid",
            "position_status",
            "position_updates",
            "move_direction",
            "move_status",
        )
        packet["q9"] = {
            name: q9[name]
            for name in q9_fields
        }
    return packet


def make_stm32_feedback_packet(
    session_id,
    transport_sequence,
    device_ms,
    feedback,
):
    """Wrap one parsed STM32 control feedback frame for UDP forwarding."""
    packet = {
        "v": PROTOCOL_VERSION,
        "type": "stm32_feedback",
        "session": str(session_id),
        "transport_seq": int(transport_sequence),
        "device_ms": int(device_ms),
    }
    fields = (
        "seq",
        "seq_gap",
        "mcu_ms",
        "vision_frame",
        "vision_age_ms",
        "position_x10",
        "velocity_x10",
        "error_x10",
        "p_x100",
        "i_x100",
        "d_x100",
        "position_px",
        "velocity_px_s",
        "control_error_px",
        "p_term",
        "i_term",
        "d_term",
        "motor_command",
        "motor_status",
        "motor_status_name",
        "raw_line",
    )
    for name in fields:
        packet[name] = feedback[name]
    return packet


def make_status_packet(
    session_id,
    sequence,
    device_ms,
    state,
    rtsp_url,
    control_port,
    camera_size,
    stream_size,
    stream_fps,
    stream_bitrate,
    roi,
    axis_start,
    axis_end,
    current_config,
    network_errors=0,
    control_errors=0,
):
    return {
        "v": PROTOCOL_VERSION,
        "type": "status",
        "session": str(session_id),
        "seq": int(sequence),
        "device_ms": int(device_ms),
        "state": str(state),
        "rtsp_url": str(rtsp_url),
        "control_port": int(control_port),
        "camera_size": list(camera_size),
        "stream_size": list(stream_size),
        "stream_fps": int(stream_fps),
        "stream_bitrate": int(stream_bitrate),
        "roi": list(roi),
        "axis_start": list(axis_start),
        "axis_end": list(axis_end),
        "config": dict(current_config),
        "network_errors": int(network_errors),
        "control_errors": int(control_errors),
    }


def make_set_config_request(request_id, token, params):
    return {
        "v": PROTOCOL_VERSION,
        "type": "set_config",
        "request_id": str(request_id),
        "token": str(token),
        "params": dict(params),
    }


def make_subscribe_request(request_id, token, telemetry_port):
    return {
        "v": PROTOCOL_VERSION,
        "type": "subscribe",
        "request_id": str(request_id),
        "token": str(token),
        "telemetry_port": int(telemetry_port),
    }


def parse_subscribe_request(data, expected_token):
    packet = decode_packet(data)
    if packet.get("type") != "subscribe":
        raise ProtocolError("invalid_type", "expected subscribe packet")
    request_id = packet.get("request_id")
    if not isinstance(request_id, str) or not request_id:
        raise ProtocolError("invalid_request_id", "request_id is required")
    if len(request_id) > 64:
        raise ProtocolError("invalid_request_id", "request_id is too long")
    if packet.get("token") != expected_token:
        raise ProtocolError("unauthorized", "control token mismatch")
    telemetry_port = packet.get("telemetry_port")
    if (
        isinstance(telemetry_port, bool)
        or not isinstance(telemetry_port, int)
        or telemetry_port < 1024
        or telemetry_port > 65535
    ):
        raise ProtocolError(
            "invalid_telemetry_port",
            "telemetry_port must be in [1024, 65535]",
        )
    return request_id, telemetry_port


def make_subscribe_ack(session_id, request_id, ok, telemetry_port, error=""):
    return {
        "v": PROTOCOL_VERSION,
        "type": "subscribe_ack",
        "session": str(session_id),
        "request_id": str(request_id),
        "ok": bool(ok),
        "telemetry_port": int(telemetry_port),
        "error": str(error),
    }


def parse_set_config_request(data, expected_token):
    packet = decode_packet(data)
    if packet.get("type") != "set_config":
        raise ProtocolError("invalid_type", "expected set_config packet")
    request_id = packet.get("request_id")
    if not isinstance(request_id, str) or not request_id:
        raise ProtocolError("invalid_request_id", "request_id is required")
    if len(request_id) > 64:
        raise ProtocolError("invalid_request_id", "request_id is too long")
    if packet.get("token") != expected_token:
        raise ProtocolError("unauthorized", "control token mismatch")
    clean, errors = validate_parameters(packet.get("params"))
    return request_id, clean, errors


def make_config_ack(
    session_id,
    request_id,
    ok,
    applied,
    errors,
    current_config,
):
    return {
        "v": PROTOCOL_VERSION,
        "type": "config_ack",
        "session": str(session_id),
        "request_id": str(request_id),
        "ok": bool(ok),
        "applied": dict(applied),
        "errors": dict(errors),
        "config": dict(current_config),
    }
