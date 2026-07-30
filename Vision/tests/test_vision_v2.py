import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from vision_v2 import (
    Axis,
    BallVisionV2,
    Candidate,
    Point,
    TrackerV2,
    VisionV2Config,
)
from stream_protocol import ProtocolError, apply_parameters, config_snapshot


class FakeBlob:
    def __init__(self, x, y, width, height, pixels, roundness=0.85):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._pixels = pixels
        self._roundness = roundness

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._width

    def h(self):
        return self._height

    def pixels(self):
        return self._pixels

    def cxf(self):
        return self._x + self._width / 2.0

    def cyf(self):
        return self._y + self._height / 2.0

    def roundness(self):
        return self._roundness


class FakeImage:
    def __init__(self):
        self.pipe_blobs = [FakeBlob(180, 45, 10, 10, 90)]
        self.ball_blobs = [FakeBlob(81, 41, 18, 18, 210)]
        self.ball_rois = []

    def find_blobs(self, thresholds, **kwargs):
        if thresholds[0][0] == 5:
            return list(self.pipe_blobs)
        self.ball_rois.append(tuple(kwargs["roi"]))
        return list(self.ball_blobs)


def make_config():
    return VisionV2Config(
        frame_width=200,
        frame_height=100,
        left_endpoint=(10, 50),
        right_endpoint=(190, 50),
        right_search_roi=(170, 20, 30, 60),
        pipe_thresholds=((5, 90, -55, -12, -15, 40),),
        ball_thresholds=((0, 85, -22, 22, -20, 20),),
        ball_diameter_px=20,
        target_position=0.5,
    )


class VisionV2Tests(unittest.TestCase):
    def test_internal_models_have_named_slots_not_open_dictionaries(self):
        config = make_config()
        pipeline = BallVisionV2(config)

        self.assertFalse(hasattr(config, "__dict__"))
        self.assertFalse(hasattr(pipeline.pipe, "__dict__"))
        self.assertFalse(hasattr(pipeline.tracker, "__dict__"))
        self.assertEqual(len(VisionV2Config.__slots__), 9)

    def test_two_hits_confirm_and_switch_to_local_search(self):
        image = FakeImage()
        pipeline = BallVisionV2(make_config())

        _, first = pipeline.process(image, 0, 0)
        detection, second = pipeline.process(image, 20, 1)

        self.assertFalse(first["valid"])
        self.assertTrue(second["valid"])
        self.assertTrue(second["measured"])
        self.assertTrue(detection["used_local"])
        self.assertLess(image.ball_rois[1][2], image.ball_rois[0][2])
        self.assertAlmostEqual(second["position_px"], 80.0, delta=1.0)
        self.assertAlmostEqual(second["error_px"], -10.0, delta=1.0)

    def test_tentative_track_uses_lower_follow_threshold(self):
        image = FakeImage()
        pipeline = BallVisionV2(make_config())
        pipeline.process(image, 0, 0)
        image.ball_blobs = [FakeBlob(82, 42, 16, 16, 34)]

        _, second = pipeline.process(image, 20, 1)

        self.assertTrue(second["valid"])
        self.assertTrue(second["measured"])

    def test_one_miss_coasts_then_next_frame_uses_broad_search(self):
        image = FakeImage()
        pipeline = BallVisionV2(make_config())
        pipeline.process(image, 0, 0)
        pipeline.process(image, 20, 1)

        image.ball_blobs = []
        _, missed = pipeline.process(image, 40, 2)
        image.ball_blobs = [FakeBlob(91, 41, 18, 18, 210)]
        detection, recovered = pipeline.process(image, 60, 3)

        self.assertTrue(missed["valid"])
        self.assertTrue(missed["coasting"])
        self.assertFalse(detection["used_local"])
        self.assertTrue(detection["fell_back"])
        self.assertTrue(recovered["measured"])

    def test_pose_expires_instead_of_exporting_stale_geometry(self):
        image = FakeImage()
        pipeline = BallVisionV2(make_config())
        pipeline.process(image, 0, 0)
        pipeline.process(image, 20, 1)
        image.pipe_blobs = []

        state = None
        detection = None
        for frame_id in range(2, 15):
            detection, state = pipeline.process(
                image, frame_id * 20, frame_id
            )

        self.assertFalse(detection["pipe"]["valid"])
        self.assertFalse(state["valid"])
        self.assertFalse(state["measured"])

    def test_startup_requires_one_real_pipe_measurement(self):
        image = FakeImage()
        image.pipe_blobs = []
        pipeline = BallVisionV2(make_config())

        detection, state = pipeline.process(image, 0, 0)

        self.assertFalse(detection["pipe"]["valid"])
        self.assertFalse(state["valid"])

    def test_tracker_prefers_continuity_over_brighter_distractor(self):
        axis = Axis((0, 0), (200, 0))
        tracker = TrackerV2(axis, 0.5)
        near = Candidate(Point(50, 0), 10, 100, 50, 0, None)
        tracker.update([near], 0, 20)
        tracker.update([near], 20, 20)

        nearby = Candidate(Point(55, 0), 10, 80, 55, 0, None)
        bright_far = Candidate(Point(150, 0), 10, 800, 150, 0, None)
        state = tracker.update([nearby, bright_far], 40, 20)

        self.assertAlmostEqual(state.measurement.point.x, 55.0)

    def test_axis_change_preserves_image_space_track(self):
        axis = Axis((0, 0), (100, 0))
        tracker = TrackerV2(axis, 0.5)
        candidate = Candidate(Point(50, 5), 10, 100, 50, 5, None)
        tracker.update([candidate], 0, 20)
        before = tracker.predicted_point(0)

        tracker.set_axis(Axis((10, 10), (210, 10)))
        after = tracker.predicted_point(0)

        self.assertAlmostEqual(before.x, after.x)
        self.assertAlmostEqual(before.y, after.y)

    def test_live_config_exposes_only_v2_calibration_surface(self):
        class ConfigModule:
            TARGET_POSITION = 0.5

        pipeline = BallVisionV2(make_config())
        snapshot = config_snapshot(pipeline, None)
        updated = apply_parameters(
            {"target_position": 0.6},
            pipeline,
            None,
            ConfigModule,
        )

        self.assertEqual(
            set(snapshot),
            {"algorithm", "target_position", "ball_diameter_px"},
        )
        self.assertEqual(updated["target_position"], 0.6)
        self.assertEqual(pipeline.tracker.target, 0.6)
        with self.assertRaises(ProtocolError):
            apply_parameters(
                {"circle_threshold": 1000},
                pipeline,
                None,
                ConfigModule,
            )


if __name__ == "__main__":
    unittest.main()
