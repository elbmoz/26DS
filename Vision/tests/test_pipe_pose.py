import math
import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from pipe_pose import (  # noqa: E402
    GreenPipePoseDetector,
    pose_from_corners,
    roi_from_axis,
)


class FakeBlob:
    def __init__(self, corners, pixels=4000):
        self._corners = corners
        self._pixels = pixels

    def mini_corners(self):
        return self._corners

    def pixels(self):
        return self._pixels


class FakeImage:
    def __init__(self, frames):
        self.frames = list(frames)
        self.calls = 0
        self.rois = []

    def find_blobs(self, *_args, **_kwargs):
        self.kwargs = _kwargs
        self.rois.append(tuple(_kwargs["roi"]))
        result = self.frames[min(self.calls, len(self.frames) - 1)]
        self.calls += 1
        return result


def rectangle_corners(center, length, width, angle_deg):
    angle = math.radians(angle_deg)
    unit = (math.cos(angle), math.sin(angle))
    normal = (-unit[1], unit[0])
    corners = []
    for along, across in (
        (-length / 2, -width / 2),
        (length / 2, -width / 2),
        (length / 2, width / 2),
        (-length / 2, width / 2),
    ):
        corners.append(
            (
                center[0] + along * unit[0] + across * normal[0],
                center[1] + along * unit[1] + across * normal[1],
            )
        )
    return corners


class PipePoseTests(unittest.TestCase):
    def test_pose_uses_major_axis_without_corner_order_dependency(self):
        corners = rectangle_corners((320, 170), 420, 28, -6)
        shuffled = [corners[2], corners[0], corners[3], corners[1]]
        pose = pose_from_corners(shuffled)
        self.assertAlmostEqual(pose["center"][0], 320, places=5)
        self.assertAlmostEqual(pose["center"][1], 170, places=5)
        self.assertAlmostEqual(pose["length"], 420, places=5)
        self.assertAlmostEqual(pose["width"], 28, places=5)
        self.assertAlmostEqual(
            math.degrees(pose["angle_rad"]), -6, places=5
        )
        self.assertLess(pose["start"][0], pose["end"][0])

    def test_dynamic_roi_encloses_tilted_axis_and_stays_in_frame(self):
        roi = roi_from_axis(
            (5, 20),
            (620, 100),
            640,
            480,
            along_margin_px=35,
            lateral_margin_px=42,
        )
        self.assertEqual(roi[0], 0)
        self.assertGreater(roi[2], 620)
        self.assertGreater(roi[3], 100)
        self.assertLessEqual(roi[0] + roi[2], 640)
        self.assertLessEqual(roi[1] + roi[3], 480)

    def test_control_roi_stays_close_to_physical_pipe_axis(self):
        roi = roi_from_axis(
            (52, 110),
            (390, 110),
            480,
            360,
            along_margin_px=3,
            lateral_margin_px=12,
        )
        self.assertEqual(roi, (49, 98, 345, 25))

    def test_detector_updates_every_second_frame_and_keeps_pose_between(self):
        first = FakeBlob(rectangle_corners((315, 168), 423, 28, 2))
        second = FakeBlob(rectangle_corners((320, 180), 423, 28, 4))
        image = FakeImage([[first], [second]])
        detector = GreenPipePoseDetector(
            640,
            480,
            (40, 80, 560, 200),
            (80, 125, 470, 85),
            (105, 161),
            (520, 175),
            ((5, 90, -55, -12, -15, 40),),
            detect_interval_frames=2,
            min_length_px=260,
            max_length_px=580,
            smoothing_alpha=1.0,
            axis_inset_px=4,
        )
        measured = detector.update(image, 0)
        skipped = detector.update(image, 1)
        moved = detector.update(image, 2)
        self.assertTrue(measured["measured"])
        self.assertFalse(skipped["measured"])
        self.assertTrue(skipped["valid"])
        self.assertEqual(skipped["age_frames"], 1)
        self.assertTrue(moved["measured"])
        self.assertEqual(image.calls, 2)
        self.assertGreater(moved["axis_start"][1], measured["axis_start"][1])
        self.assertNotEqual(moved["ball_roi"], measured["ball_roi"])

    def test_car_mode_does_not_merge_green_chassis_components(self):
        pipe = FakeBlob(rectangle_corners((300, 145), 430, 45, 2))
        image = FakeImage([[pipe]])
        detector = GreenPipePoseDetector(
            640,
            480,
            (25, 55, 590, 190),
            (40, 112, 570, 68),
            (70, 145),
            (590, 145),
            ((5, 90, -55, -12, -15, 40),),
            merge_blobs=False,
        )
        self.assertTrue(detector.update(image, 0)["measured"])
        self.assertFalse(image.kwargs["merge"])

    def test_adaptive_pose_miss_delays_broad_retry(self):
        pipe = FakeBlob(rectangle_corners((315, 168), 423, 28, 2))
        image = FakeImage([[pipe], [], [], [pipe]])
        detector = GreenPipePoseDetector(
            640,
            480,
            (40, 80, 560, 200),
            (80, 125, 470, 85),
            (105, 161),
            (520, 175),
            ((5, 90, -55, -12, -15, 40),),
            detect_interval_frames=1,
            min_length_px=260,
            max_length_px=580,
            smoothing_alpha=1.0,
            broad_retry_interval_updates=2,
        )

        locked = detector.update(image, 0)
        first_miss = detector.update(image, 1)
        recovered = detector.update(image, 2)

        self.assertTrue(locked["measured"])
        self.assertFalse(first_miss["measured"])
        self.assertFalse(first_miss["fell_back"])
        self.assertTrue(first_miss["valid"])
        self.assertTrue(recovered["measured"])
        self.assertTrue(recovered["fell_back"])
        self.assertEqual(image.calls, 4)

    def test_fixed_search_mode_never_expands_after_lock(self):
        pipe = FakeBlob(rectangle_corners((320, 168), 180, 24, 2))
        fixed_roi = (230, 135, 180, 66)
        image = FakeImage([[pipe], []])
        detector = GreenPipePoseDetector(
            640,
            480,
            fixed_roi,
            (80, 125, 470, 85),
            (105, 161),
            (520, 175),
            ((5, 90, -55, -12, -15, 40),),
            detect_interval_frames=1,
            min_length_px=60,
            max_length_px=240,
            broad_retry_interval_updates=1,
            fixed_search_roi=True,
        )

        locked = detector.update(image, 0)
        missed = detector.update(image, 1)

        self.assertTrue(locked["measured"])
        self.assertFalse(missed["measured"])
        self.assertFalse(missed["fell_back"])
        self.assertEqual(image.calls, 2)
        self.assertEqual(image.rois, [fixed_roi, fixed_roi])

    def test_fixed_car_geometry_uses_blob_angle_not_fragment_endpoints(self):
        fragment = FakeBlob(
            rectangle_corners((270, 135), 300, 35, 8), pixels=2500
        )
        image = FakeImage([[fragment]])
        detector = GreenPipePoseDetector(
            640,
            480,
            (25, 55, 590, 190),
            (40, 112, 570, 68),
            (70, 145),
            (590, 145),
            ((5, 90, -55, -12, -15, 40),),
            fixed_axis_center=(330, 145),
            fixed_axis_length_px=520,
            smoothing_alpha=1.0,
        )
        state = detector.update(image, 0)
        center_x = 0.5 * (
            state["axis_start"][0] + state["axis_end"][0]
        )
        center_y = 0.5 * (
            state["axis_start"][1] + state["axis_end"][1]
        )
        length = math.hypot(
            state["axis_end"][0] - state["axis_start"][0],
            state["axis_end"][1] - state["axis_start"][1],
        )
        self.assertAlmostEqual(center_x, 330)
        self.assertAlmostEqual(center_y, 145)
        self.assertAlmostEqual(length, 520)

    def test_rejects_green_distractor_far_from_car_pivot(self):
        distractor = FakeBlob(
            rectangle_corners((100, 300), 400, 40, 0), pixels=5000
        )
        image = FakeImage([[distractor]])
        detector = GreenPipePoseDetector(
            640,
            480,
            (0, 0, 640, 480),
            (40, 112, 570, 68),
            (70, 145),
            (590, 145),
            ((5, 90, -55, -12, -15, 40),),
            expected_center=(330, 145),
            max_center_distance_px=135,
        )
        self.assertFalse(detector.update(image, 0)["measured"])


if __name__ == "__main__":
    unittest.main()
