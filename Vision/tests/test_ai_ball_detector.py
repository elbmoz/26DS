import sys
import json
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from ai_ball_detector import AIBallDetector, AIVisionConfig
from stream_protocol import config_snapshot, make_tracking_packet


class FakeObject:
    def __init__(self, x, y, w, h, score=0.8, class_id=0):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.score = score
        self.class_id = class_id


class FakeModel:
    labels = ("steel-ball",)

    def __init__(self):
        self.objects = []
        self.calls = []

    def input_width(self):
        return 224

    def input_height(self):
        return 224

    def detect(self, img, conf_th, iou_th):
        self.calls.append((img, conf_th, iou_th))
        return list(self.objects)


class FakeImage:
    def __init__(self):
        self.resize_calls = []

    def resize(self, width, height):
        self.resize_calls.append((width, height))
        return ("resized", width, height)


def make_detector(model=None, runtime_config_path=None):
    config = AIVisionConfig(
        model_path="/root/model.mud",
        frame_width=448,
        frame_height=336,
        axis_start=(20, 100),
        axis_end=(428, 100),
        target_position=0.5,
        confidence=0.25,
        valid_confidence=0.50,
        iou=0.45,
        coast_frames=2,
        runtime_config_path=runtime_config_path,
    )
    return AIBallDetector(config, model=model or FakeModel())


class AIBallDetectorTests(unittest.TestCase):
    def test_maps_yolo_box_back_to_camera_coordinates(self):
        model = FakeModel()
        model.objects = [FakeObject(100, 60, 12, 14, 0.75)]
        detector = make_detector(model)
        image = FakeImage()

        detection, state = detector.process(image, 1000, 0)

        self.assertEqual(image.resize_calls, [(224, 224)])
        self.assertEqual(model.calls[0][1:], (0.25, 0.45))
        box = detection["boxes"][0]
        self.assertEqual(box[:4], (200.0, 90.0, 24.0, 21.0))
        self.assertAlmostEqual(state["measurement_x"], 212.0)
        self.assertAlmostEqual(state["measurement_y"], 100.5)
        self.assertTrue(state["valid"])
        self.assertTrue(state["measured"])
        self.assertAlmostEqual(state["quality"], 75.0)

    def test_uses_highest_confidence_detection_without_geometry_rejects(self):
        model = FakeModel()
        model.objects = [
            FakeObject(10, 20, 10, 10, 0.51),
            FakeObject(180, 30, 12, 12, 0.91),
        ]
        detector = make_detector(model)

        _detection, state = detector.process(FakeImage(), 1000, 0)

        self.assertAlmostEqual(state["measurement_x"], 372.0)
        self.assertAlmostEqual(state["quality"], 91.0)

    def test_coasts_for_two_frames_then_reports_lost(self):
        model = FakeModel()
        model.objects = [FakeObject(100, 60, 12, 14, 0.75)]
        detector = make_detector(model)
        detector.process(FakeImage(), 1000, 0)
        model.objects = []

        _detection, first = detector.process(FakeImage(), 1033, 1)
        _detection, second = detector.process(FakeImage(), 1066, 2)
        _detection, third = detector.process(FakeImage(), 1099, 3)

        self.assertTrue(first["valid"])
        self.assertTrue(first["coasting"])
        self.assertTrue(second["valid"])
        self.assertFalse(third["valid"])

    def test_dynamic_pipe_axis_keeps_filtered_image_point_stable(self):
        model = FakeModel()
        model.objects = [FakeObject(100, 60, 12, 14, 0.75)]
        detector = make_detector(model)
        detector.process(FakeImage(), 1000, 0)
        old_axis = detector.pipe.axis
        old_point = old_axis.point(
            detector.position_px, detector.lateral_px
        )
        model.objects = []
        pipe_state = {
            "axis_start": (20, 100),
            "axis_end": (428, 108),
            "ball_roi": (8, 70, 430, 70),
            "ball_quad": ((8, 70), (438, 78), (438, 138), (8, 130)),
            "measured": True,
            "valid": True,
            "age_frames": 0,
            "raw_blob_count": 1,
            "score": 900.0,
            "length": 82.0,
            "width": 14.0,
            "mode": "fixed_left_endpoint",
        }

        detection, _state = detector.process(
            FakeImage(), 1001, 1, pipe_state=pipe_state
        )

        new_point = detector.pipe.axis.point(
            detector.position_px, detector.lateral_px
        )
        self.assertAlmostEqual(new_point.x, old_point.x)
        self.assertAlmostEqual(new_point.y, old_point.y)
        self.assertEqual(detection["pipe"], pipe_state)
        self.assertEqual(detection["axis_end"], (428.0, 108.0))
        self.assertEqual(detection["roi_quad"], pipe_state["ball_quad"])

    def test_low_confidence_box_is_monitor_only(self):
        model = FakeModel()
        model.objects = [FakeObject(100, 60, 12, 14, 0.35)]
        detector = make_detector(model)

        detection, state = detector.process(FakeImage(), 1000, 0)

        self.assertEqual(len(detection["boxes"]), 1)
        self.assertFalse(state["valid"])
        self.assertFalse(state["measured"])

    def test_protocol_exposes_model_and_ai_boxes(self):
        model = FakeModel()
        model.objects = [FakeObject(100, 60, 12, 14, 0.75)]
        detector = make_detector(model)
        detection, state = detector.process(FakeImage(), 1000, 0)

        snapshot = config_snapshot(detector, None)
        packet = make_tracking_packet(
            "session",
            1,
            1000,
            0,
            33,
            30.0,
            20,
            state,
            detection,
        )

        self.assertEqual(snapshot["algorithm"], "ai")
        self.assertEqual(snapshot["input_size"], [224, 224])
        self.assertEqual(snapshot["valid_confidence"], 0.5)
        self.assertEqual(packet["algorithm"], "ai")
        self.assertEqual(len(packet["ai_boxes"]), 1)
        self.assertAlmostEqual(packet["ai_boxes"][0][4], 0.75)

    def test_runtime_thresholds_are_persisted(self):
        with tempfile.TemporaryDirectory() as temporary:
            runtime_path = Path(temporary) / "ai.json"
            detector = make_detector(
                runtime_config_path=str(runtime_path)
            )

            detector.apply_runtime_config(
                {
                    "confidence": 0.13,
                    "valid_confidence": 0.13,
                    "iou": 0.4,
                    "coast_frames": 4,
                    "target_position": 0.6,
                    "travel_start_px": 14.5,
                    "travel_end_px": 17.0,
                }
            )

            saved = json.loads(runtime_path.read_text(encoding="utf-8"))
            self.assertEqual(saved["confidence"], 0.13)
            self.assertEqual(saved["valid_confidence"], 0.13)
            self.assertEqual(saved["coast_frames"], 4)
            self.assertEqual(saved["travel_start_px"], 14.5)
            self.assertEqual(saved["travel_end_px"], 17.0)
            self.assertEqual(detector.config.target_position, 0.6)

    def test_mechanical_travel_calibration_maps_ball_centres_to_limits(self):
        detector = make_detector()
        detector.apply_runtime_config(
            {"travel_start_px": 15.0, "travel_end_px": 18.0}
        )
        detector.position_px = 15.0
        detector.radius = 10.0
        left = detector._state(True)
        detector.position_px = detector.pipe.axis.length - 18.0
        right = detector._state(True)

        self.assertEqual(left["position"], 0.0)
        self.assertEqual(right["position"], 1.0)
        self.assertAlmostEqual(left["travel_position_px"], 0.0)
        self.assertAlmostEqual(
            right["travel_position_px"], right["travel_length_px"]
        )
        self.assertAlmostEqual(
            left["target_axis_px"],
            15.0 + 0.5 * left["travel_length_px"],
        )

    def test_model_switch_replaces_model_only_after_loading(self):
        detector = make_detector()
        replacement = FakeModel()
        fake_maix = types.SimpleNamespace(
            nn=types.SimpleNamespace(
                YOLOv5=lambda model: replacement
            )
        )

        with patch.dict(sys.modules, {"maix": fake_maix}):
            detector.apply_runtime_config(
                {
                    "model": (
                        "/root/models/maixhub/312328/"
                        "model_312328.mud"
                    )
                }
            )

        self.assertIs(detector.model, replacement)
        self.assertEqual(
            detector.config.model_path,
            "/root/models/maixhub/312328/model_312328.mud",
        )
        self.assertIsNone(detector.position_px)


if __name__ == "__main__":
    unittest.main()
