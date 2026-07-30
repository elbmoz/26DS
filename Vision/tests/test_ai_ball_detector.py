import sys
import unittest
from pathlib import Path


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


def make_detector(model=None):
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


if __name__ == "__main__":
    unittest.main()
