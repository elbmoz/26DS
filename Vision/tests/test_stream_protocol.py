import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from stream_protocol import (
    ProtocolError,
    apply_parameters,
    config_snapshot,
    decode_packet,
    encode_packet,
    make_set_config_request,
    make_pid_request,
    make_stm32_feedback_packet,
    make_subscribe_request,
    make_tracking_packet,
    parse_subscribe_request,
    parse_set_config_request,
    parse_pid_request,
    validate_parameters,
)


class FakeDetector:
    local_width = 140
    circle_threshold = 1800
    circle_min_radius = 11
    circle_max_radius = 18


class FakeTracker:
    target_position = 0.5
    position_alpha = 0.72
    velocity_beta = 0.14
    lateral_alpha = 0.55
    max_axis_distance_px = 22.0
    max_below_axis_distance_px = 12.0
    max_frame_jump_px = 95.0
    acquire_position_margin = 0.08
    track_position_margin = 0.02
    acquire_endpoint_inset = 0.02
    track_endpoint_inset = 0.015
    acquire_min_quality = 60.0
    track_min_quality = 80.0
    coast_frames = 3
    memory_frames = 8


class FakeConfig:
    TARGET_POSITION = 0.5
    POSITION_ALPHA = 0.72
    VELOCITY_BETA = 0.14
    LATERAL_ALPHA = 0.55
    MAX_AXIS_DISTANCE_PX = 22.0
    MAX_BELOW_AXIS_DISTANCE_PX = 12.0
    MAX_FRAME_JUMP_PX = 95.0
    ACQUIRE_POSITION_MARGIN = 0.08
    TRACK_POSITION_MARGIN = 0.02
    ACQUIRE_MIN_QUALITY = 60.0
    TRACK_MIN_QUALITY = 80.0
    COAST_FRAMES = 3
    LOCAL_SEARCH_WIDTH_PX = 140
    CIRCLE_THRESHOLD = 1800
    CIRCLE_MIN_RADIUS = 11
    CIRCLE_MAX_RADIUS = 18


class StreamProtocolTests(unittest.TestCase):
    def test_tracking_packet_is_compact_and_round_trips(self):
        state = {
            "valid": True,
            "measured": True,
            "coasting": False,
            "x": 320.1256,
            "y": 165.25,
            "radius": 9.5,
            "position": 0.51512,
            "position_px": 211.2,
            "error_px": 4,
            "lateral_px": -1,
            "velocity_px_s": 182.3,
            "quality": 91.2,
            "measurement_x": 321.0,
            "measurement_y": 165.0,
            "hits": 4,
            "misses": 0,
        }
        detection = {
            "raw_count": 2,
            "candidates": [(321, 165, 9, 91)],
            "search_roi": (250, 125, 140, 85),
            "full_roi": (80, 125, 470, 85),
            "fell_back": False,
            "axis_start": (102.5, 150.25),
            "axis_end": (521.5, 178.0),
            "roi_quad": (
                (80.0, 125.0),
                (550.0, 140.0),
                (550.0, 210.0),
                (80.0, 210.0),
            ),
            "pipe": {
                "measured": True,
                "valid": True,
                "age_frames": 0,
                "raw_blob_count": 1,
                "length": 427.0,
                "width": 28.0,
                "score": 12345.0,
            },
        }
        packet = make_tracking_packet(
            "session",
            12,
            500,
            25,
            20,
            51.2,
            7,
            state,
            detection,
        )
        encoded = encode_packet(packet)
        self.assertLess(len(encoded), 1200)
        decoded = decode_packet(encoded)
        self.assertEqual(decoded["type"], "tracking")
        self.assertEqual(decoded["frame_id"], 25)
        self.assertTrue(decoded["local_search"])
        self.assertEqual(decoded["axis_x0"], 102.5)
        self.assertTrue(decoded["pipe_valid"])
        self.assertEqual(decoded["roi_w"], 470)
        self.assertEqual(len(decoded["roi_quad"]), 4)

    def test_tracking_packet_can_forward_latest_q9_snapshot(self):
        state = {
            "valid": False,
            "measured": False,
            "coasting": False,
            "x": 0,
            "y": 0,
            "radius": 0,
            "position": 0,
            "position_px": 0,
            "error_px": 0,
            "lateral_px": 0,
            "velocity_px_s": 0,
            "quality": 0,
            "hits": 0,
            "misses": 0,
        }
        detection = {
            "raw_count": 0,
            "candidates": [],
            "search_roi": (0, 0, 10, 10),
            "fell_back": False,
            "pipe": {},
        }
        q9 = {
            "seq": 17,
            "seq_gap": 0,
            "mcu_ms": 25340,
            "motor_position": 4897,
            "angle_x_x10": -123,
            "angle_y_x10": 48,
            "angle_z_x10": 906,
            "angle_x_deg": -12.3,
            "angle_y_deg": 4.8,
            "angle_z_deg": 90.6,
            "imu_valid": 1,
            "position_valid": 1,
            "position_status": 0,
            "position_updates": 84,
            "move_direction": -1,
            "move_status": 0,
        }
        packet = make_tracking_packet(
            "session",
            1,
            2,
            3,
            4,
            5,
            6,
            state,
            detection,
            q9=q9,
        )
        decoded = decode_packet(encode_packet(packet))
        self.assertEqual(decoded["q9"]["motor_position"], 4897)
        self.assertEqual(decoded["q9"]["angle_z_deg"], 90.6)
        self.assertNotIn("raw_line", decoded["q9"])

    def test_stm32_feedback_packet_preserves_mcu_sequence(self):
        feedback = {
            "seq": 17,
            "seq_gap": 2,
            "mcu_ms": 1234,
            "vision_frame": 88,
            "vision_age_ms": 7,
            "position_x10": 15,
            "velocity_x10": -25,
            "error_x10": 35,
            "p_x100": 120,
            "i_x100": -20,
            "d_x100": 5,
            "position_px": 1.5,
            "velocity_px_s": -2.5,
            "control_error_px": 3.5,
            "p_term": 1.2,
            "i_term": -0.2,
            "d_term": 0.05,
            "motor_command": -320,
            "motor_status": 2,
            "motor_status_name": "HAL_BUSY",
            "raw_line": "F,17,1234,88,7,15,-25,35,120,-20,5,-320,2",
        }
        packet = make_stm32_feedback_packet(
            "session", 99, 500, feedback
        )
        decoded = decode_packet(encode_packet(packet))
        self.assertEqual(decoded["type"], "stm32_feedback")
        self.assertEqual(decoded["transport_seq"], 99)
        self.assertEqual(decoded["seq"], 17)
        self.assertEqual(decoded["motor_command"], -320)

    def test_parameter_validation_rejects_unknown_and_out_of_range(self):
        clean, errors = validate_parameters(
            {
                "target_position": 1.2,
                "coast_frames": 2.5,
                "unknown": 10,
                "velocity_beta": 0.2,
            }
        )
        self.assertEqual(clean, {"velocity_beta": 0.2})
        self.assertIn("target_position", errors)
        self.assertIn("coast_frames", errors)
        self.assertIn("unknown", errors)

    def test_pid_request_validates_action_parameters_and_token(self):
        request = make_pid_request(
            "p1",
            "secret",
            "set",
            {"inner_kp": 4.0, "speed_limit": 30, "outer_ki": 0.012},
        )
        request_id, action, params = parse_pid_request(
            encode_packet(request), "secret"
        )
        self.assertEqual((request_id, action), ("p1", "set"))
        self.assertEqual(params["inner_kp"], 4.0)
        self.assertEqual(params["outer_ki"], 0.012)

        with self.assertRaises(ProtocolError):
            parse_pid_request(encode_packet(request), "wrong")
        request["params"] = {"motor_direction": -1}
        with self.assertRaises(ProtocolError):
            parse_pid_request(encode_packet(request), "secret")

    def test_pid_profile_validates_mcu_timed_experiment(self):
        request = make_pid_request(
            "profile-1",
            "secret",
            "profile",
            {
                "mode": "outer",
                "amplitude": 50,
                "phase_ms": 1600,
                "settle_band": 12,
                "settle_rate": 30,
                "settle_ms": 160,
            },
        )
        request_id, action, params = parse_pid_request(
            encode_packet(request), "secret"
        )
        self.assertEqual((request_id, action), ("profile-1", "profile"))
        self.assertEqual(params["phase_ms"], 1600)
        self.assertEqual(params["settle_band"], 12.0)

    def test_ai_model_and_threshold_validation(self):
        model = "/root/models/maixhub/312328/model_312328.mud"
        clean, errors = validate_parameters(
            {
                "model": model,
                "confidence": 0.13,
                "valid_confidence": 0.13,
                "iou": 0.45,
                "travel_start_px": 14.5,
                "travel_end_px": 18,
            }
        )
        self.assertFalse(errors)
        self.assertEqual(clean["model"], model)
        self.assertEqual(clean["confidence"], 0.13)
        self.assertEqual(clean["travel_start_px"], 14.5)
        self.assertEqual(clean["travel_end_px"], 18)

        clean, errors = validate_parameters(
            {
                "model": "/root/other/model.mud",
                "confidence": 0.0,
            }
        )
        self.assertFalse(clean)
        self.assertIn("model", errors)
        self.assertIn("confidence", errors)

    def test_apply_parameters_updates_runtime_objects(self):
        detector = FakeDetector()
        tracker = FakeTracker()
        config = FakeConfig()
        clean, errors = validate_parameters(
            {
                "target_position": 0.55,
                "position_alpha": 0.8,
                "max_below_axis_distance_px": 14,
                "acquire_position_margin": 0.07,
                "track_position_margin": 0.03,
                "acquire_endpoint_inset": 0.025,
                "track_endpoint_inset": 0.01,
                "acquire_min_quality": 70,
                "track_min_quality": 75,
                "coast_frames": 5,
                "local_search_width_px": 180,
                "circle_threshold": 700,
            }
        )
        self.assertFalse(errors)
        result = apply_parameters(clean, detector, tracker, config)
        self.assertEqual(result["target_position"], 0.55)
        self.assertEqual(tracker.position_alpha, 0.8)
        self.assertEqual(tracker.max_below_axis_distance_px, 14)
        self.assertEqual(tracker.acquire_position_margin, 0.07)
        self.assertEqual(tracker.track_position_margin, 0.03)
        self.assertEqual(tracker.acquire_endpoint_inset, 0.025)
        self.assertEqual(tracker.track_endpoint_inset, 0.01)
        self.assertEqual(tracker.acquire_min_quality, 70)
        self.assertEqual(tracker.track_min_quality, 75)
        self.assertEqual(tracker.coast_frames, 5)
        self.assertEqual(detector.local_width, 180)
        self.assertEqual(detector.circle_threshold, 700)
        self.assertEqual(config.TARGET_POSITION, 0.55)

    def test_control_request_requires_matching_token(self):
        request = make_set_config_request(
            "req-1", "correct", {"target_position": 0.6}
        )
        request_id, clean, errors = parse_set_config_request(
            encode_packet(request), "correct"
        )
        self.assertEqual(request_id, "req-1")
        self.assertEqual(clean["target_position"], 0.6)
        self.assertFalse(errors)
        with self.assertRaises(ProtocolError):
            parse_set_config_request(encode_packet(request), "wrong")

    def test_subscribe_request_validates_port_and_token(self):
        request = make_subscribe_request("sub-1", "correct", 42101)
        request_id, port = parse_subscribe_request(
            encode_packet(request), "correct"
        )
        self.assertEqual(request_id, "sub-1")
        self.assertEqual(port, 42101)
        request["telemetry_port"] = 80
        with self.assertRaises(ProtocolError):
            parse_subscribe_request(encode_packet(request), "correct")

    def test_config_snapshot_contains_only_runtime_safe_fields(self):
        snapshot = config_snapshot(FakeDetector(), FakeTracker())
        self.assertIn("target_position", snapshot)
        self.assertIn("local_search_width_px", snapshot)
        self.assertNotIn("roi", snapshot)
        self.assertNotIn("motor", snapshot)


if __name__ == "__main__":
    unittest.main()
