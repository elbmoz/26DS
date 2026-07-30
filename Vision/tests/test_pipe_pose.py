import math
import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from pipe_pose import (  # noqa: E402
    GreenPipePoseDetector,
    TapeEndpointPipePoseDetector,
    pose_from_corners,
    quadrilateral_from_axis,
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


class FakeTapeBlob:
    def __init__(self, x, y, width, height, pixels, left=None):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._pixels = pixels
        self._left = (
            int(round(x - width / 2.0)) if left is None else int(left)
        )

    def x(self):
        return self._left

    def w(self):
        return self._width

    def h(self):
        return self._height

    def pixels(self):
        return self._pixels

    def cx(self):
        return self._x

    def cy(self):
        return self._y


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
    def test_quadrilateral_matches_rotated_effective_strip(self):
        quad = quadrilateral_from_axis(
            (10, 20),
            (90, 20),
            100,
            80,
            along_margin_px=0,
            lateral_margin_px=5,
        )

        self.assertEqual(
            quad,
            (
                (10.0, 25.0),
                (90.0, 25.0),
                (90.0, 15.0),
                (10.0, 15.0),
            ),
        )

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

    def test_control_roi_can_extend_only_the_fixed_left_end(self):
        roi = roi_from_axis(
            (26, 99),
            (397, 108),
            480,
            360,
            along_margin_px=0,
            lateral_margin_px=12,
            start_along_margin_px=12,
            end_along_margin_px=0,
        )
        self.assertEqual(roi[0], 13)
        self.assertEqual(roi[0] + roi[2], 399)

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

    def test_right_tape_directly_defines_pipe_endpoints(self):
        tape = FakeTapeBlob(390, 114, 22, 29, 310)
        image = FakeImage([[tape]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(368, 52, 42, 94),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 94),
            fallback_right_endpoint=(390, 114),
            thresholds=((0, 30, -20, 20, -20, 20),),
            detect_interval_frames=3,
            min_width_px=8,
            max_width_px=32,
            min_height_px=18,
            max_height_px=48,
            min_pixels=45,
            expected_right_x=390,
            max_right_x_distance_px=21,
            min_axis_length_px=322,
            max_axis_length_px=402,
            smoothing_alpha=1.0,
            roi_along_margin_px=0,
            roi_lateral_margin_px=12,
        )

        state = detector.update(image, 0)

        self.assertTrue(state["measured"])
        self.assertTrue(state["valid"])
        self.assertEqual(state["axis_start"], (26.0, 94.0))
        self.assertEqual(state["axis_end"], (390.0, 114.0))
        self.assertEqual(state["ball_roi"], (25, 82, 367, 45))
        self.assertEqual(image.rois, [(368, 52, 42, 94)])

    def test_right_tape_detector_rejects_short_background_fragment(self):
        background = FakeTapeBlob(390, 82, 20, 12, 90)
        image = FakeImage([[background]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(368, 52, 42, 94),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 94),
            fallback_right_endpoint=(390, 114),
            thresholds=((0, 30, -20, 20, -20, 20),),
            min_height_px=18,
            min_axis_length_px=322,
            max_axis_length_px=402,
        )

        state = detector.update(image, 0)

        self.assertFalse(state["measured"])
        self.assertFalse(state["valid"])

    def test_right_tape_pose_is_smoothed_and_rate_limited(self):
        first = FakeTapeBlob(390, 114, 22, 29, 310)
        moved = FakeTapeBlob(390, 126, 22, 29, 310)
        image = FakeImage([[first], [moved]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(368, 52, 42, 94),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 94),
            fallback_right_endpoint=(390, 114),
            thresholds=((0, 30, -20, 20, -20, 20),),
            detect_interval_frames=3,
            min_axis_length_px=322,
            max_axis_length_px=402,
            smoothing_alpha=0.5,
        )

        first_state = detector.update(image, 0)
        skipped = detector.update(image, 1)
        moved_state = detector.update(image, 3)

        self.assertTrue(first_state["measured"])
        self.assertFalse(skipped["measured"])
        self.assertEqual(image.calls, 2)
        self.assertAlmostEqual(moved_state["axis_end"][1], 120.0)

    def test_green_blob_right_edge_marks_black_tape_boundary(self):
        green_fragment = FakeTapeBlob(
            356,
            108,
            48,
            15,
            420,
            left=332,
        )
        image = FakeImage([[green_fragment]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(319, 71, 90, 79),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 99),
            fallback_right_endpoint=(389, 108),
            thresholds=((5, 90, -55, -12, -15, 40),),
            min_width_px=22,
            max_width_px=112,
            min_height_px=6,
            max_height_px=36,
            min_pixels=34,
            expected_right_x=389,
            max_right_x_distance_px=21,
            min_axis_length_px=322,
            max_axis_length_px=402,
            endpoint_from_blob_right_edge=True,
            endpoint_x_offset_px=10,
            smoothing_alpha=1.0,
        )

        state = detector.update(image, 0)

        self.assertTrue(state["measured"])
        self.assertEqual(state["axis_end"], (389.0, 108.0))

    def test_right_tape_is_projected_onto_fixed_trajectory(self):
        green_fragment = FakeTapeBlob(
            360,
            108,
            42,
            15,
            420,
            left=352,
        )
        image = FakeImage([[green_fragment]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(319, 71, 90, 79),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 99),
            fallback_right_endpoint=(397, 108),
            thresholds=((5, 90, -55, -12, -15, 40),),
            min_width_px=22,
            max_width_px=112,
            min_height_px=6,
            max_height_px=36,
            min_pixels=34,
            expected_right_x=393,
            max_right_x_distance_px=2,
            fixed_right_x=397,
            min_axis_length_px=322,
            max_axis_length_px=402,
            endpoint_from_blob_right_edge=True,
            smoothing_alpha=1.0,
            roi_start_margin_px=12,
        )

        state = detector.update(image, 0)

        self.assertTrue(state["measured"])
        self.assertEqual(state["axis_start"], (26.0, 99.0))
        self.assertEqual(state["axis_end"], (397.0, 108.0))
        self.assertEqual(state["ball_roi"][0], 13)

    def test_outer_green_edge_wins_over_internal_tape_shadow(self):
        shadow = FakeTapeBlob(
            374,
            93,
            48,
            24,
            500,
            left=350,
        )
        outer_green = FakeTapeBlob(
            407,
            93,
            33,
            24,
            360,
            left=390,
        )
        image = FakeImage([[shadow, outer_green]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(390, 68, 50, 82),
            fallback_roi=(22, 79, 412, 42),
            fixed_left_endpoint=(26, 99),
            fallback_right_endpoint=(432, 93),
            thresholds=((5, 90, -55, -12, -15, 40),),
            min_width_px=8,
            max_width_px=68,
            min_height_px=6,
            max_height_px=36,
            min_pixels=34,
            expected_right_x=422,
            max_right_x_distance_px=9,
            fixed_right_x=432,
            min_axis_length_px=405,
            max_axis_length_px=465,
            endpoint_from_blob_right_edge=True,
            smoothing_alpha=1.0,
        )

        state = detector.update(image, 0)

        self.assertTrue(state["measured"])
        self.assertEqual(state["raw_blob_count"], 2)
        self.assertEqual(state["axis_end"], (432.0, 93.0))
        self.assertEqual(state["width"], 24.0)
        self.assertEqual(len(state["ball_quad"]), 4)

    def test_right_tape_rejects_impossible_y_jump_before_smoothing(self):
        first = FakeTapeBlob(390, 108, 22, 20, 310)
        shadow = FakeTapeBlob(390, 132, 22, 20, 310)
        image = FakeImage([[first], [shadow]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(368, 52, 42, 94),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 99),
            fallback_right_endpoint=(397, 108),
            thresholds=((5, 90, -55, -12, -15, 40),),
            detect_interval_frames=3,
            fixed_right_x=397,
            max_right_y_step_px=9,
            min_axis_length_px=322,
            max_axis_length_px=402,
            smoothing_alpha=0.5,
        )

        first_state = detector.update(image, 0)
        jumped_state = detector.update(image, 3)

        self.assertTrue(first_state["measured"])
        self.assertFalse(jumped_state["measured"])
        self.assertEqual(jumped_state["axis_end"], (397.0, 108.0))
        self.assertEqual(jumped_state["age_frames"], 1)

    def test_stale_right_tape_can_relock_after_large_real_motion(self):
        first = FakeTapeBlob(390, 108, 22, 20, 310)
        moved = FakeTapeBlob(390, 132, 22, 20, 310)
        image = FakeImage([[first], [], [], [moved]])
        detector = TapeEndpointPipePoseDetector(
            480,
            360,
            right_search_roi=(368, 52, 42, 94),
            fallback_roi=(20, 70, 390, 70),
            fixed_left_endpoint=(26, 99),
            fallback_right_endpoint=(397, 108),
            thresholds=((5, 90, -55, -12, -15, 40),),
            detect_interval_frames=1,
            fixed_right_x=397,
            max_right_y_step_px=9,
            min_axis_length_px=322,
            max_axis_length_px=402,
            smoothing_alpha=1.0,
            max_stale_frames=1,
        )

        detector.update(image, 0)
        detector.update(image, 1)
        stale_state = detector.update(image, 2)
        relocked_state = detector.update(image, 3)

        self.assertFalse(stale_state["valid"])
        self.assertTrue(relocked_state["measured"])
        self.assertTrue(relocked_state["valid"])
        self.assertEqual(relocked_state["axis_end"], (397.0, 132.0))


if __name__ == "__main__":
    unittest.main()
