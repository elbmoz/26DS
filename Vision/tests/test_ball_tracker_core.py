import sys
import unittest
from pathlib import Path


MAIXCAM_DIR = Path(__file__).resolve().parents[1] / "maixcam"
sys.path.insert(0, str(MAIXCAM_DIR))

from ball_detector import LabBallDetector, local_search_roi
from ball_tracker_core import BallTracker, format_vision_line, project_to_axis
from tracking_log import CSV_FIELDS, RunStats, csv_header, tracking_row
from loop_timing import periodic_due


class FakeBlob:
    def __init__(self, x, y, w, h, pixels, roundness=0.8):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._pixels = pixels
        self._roundness = roundness

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._w

    def h(self):
        return self._h

    def pixels(self):
        return self._pixels

    def cxf(self):
        return self._x + self._w / 2

    def cyf(self):
        return self._y + self._h / 2

    def roundness(self):
        return self._roundness


class FakeCircle:
    def __init__(self, x, y, radius, magnitude=2400):
        self._x = x
        self._y = y
        self._radius = radius
        self._magnitude = magnitude

    def x(self):
        return self._x

    def y(self):
        return self._y

    def r(self):
        return self._radius

    def magnitude(self):
        return self._magnitude


class BallTrackerCoreTests(unittest.TestCase):
    def test_projection_on_horizontal_axis(self):
        position, lateral, px, py, length = project_to_axis(
            60, 25, (10, 20), (110, 20)
        )
        self.assertAlmostEqual(position, 0.5)
        self.assertAlmostEqual(lateral, 5.0)
        self.assertAlmostEqual(px, 60.0)
        self.assertAlmostEqual(py, 20.0)
        self.assertAlmostEqual(length, 100.0)

    def test_requires_two_consecutive_hits(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=2,
            smoothing_alpha=1.0,
        )
        first = tracker.update([(50, 1, 10, 100)], 0)
        second = tracker.update([(55, 1, 10, 100)], 20)
        self.assertFalse(first["valid"])
        self.assertTrue(second["valid"])
        self.assertTrue(second["measured"])
        self.assertEqual(second["error_px"], 5)

    def test_rejects_candidate_far_from_pipe_axis(self):
        tracker = BallTracker(
            (0, 0), (100, 0), max_axis_distance_px=10, confirm_frames=1
        )
        state = tracker.update([(50, 30, 10, 100)], 0)
        self.assertFalse(state["valid"])

    def test_prefers_temporally_near_candidate(self):
        tracker = BallTracker(
            (0, 0),
            (200, 0),
            confirm_frames=1,
            smoothing_alpha=1.0,
        )
        tracker.update([(50, 0, 10, 100)], 0)
        state = tracker.update(
            [(55, 0, 10, 80), (150, 0, 10, 500)], 20
        )
        self.assertAlmostEqual(state["x"], 55.0)

    def test_short_miss_coasts_instead_of_immediate_none(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            coast_frames=2,
            memory_frames=4,
        )
        tracker.update([(50, 0, 10, 100)], 0)
        first_miss = tracker.update([], 20)
        second_miss = tracker.update([], 40)
        third_miss = tracker.update([], 60)
        self.assertTrue(first_miss["valid"])
        self.assertTrue(first_miss["coasting"])
        self.assertTrue(second_miss["valid"])
        self.assertFalse(third_miss["valid"])

    def test_pipe_end_reflection_cannot_start_a_track(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=2,
            acquire_min_quality=60,
        )
        for now_ms in range(0, 200, 20):
            state = tracker.update([(106, 0, 7, 20)], now_ms)
            self.assertFalse(state["valid"])
            self.assertFalse(state["measured"])

        first_ball = tracker.update([(20, 0, 11, 130)], 200)
        second_ball = tracker.update([(22, 0, 11, 125)], 220)
        self.assertFalse(first_ball["valid"])
        self.assertTrue(second_ball["valid"])
        self.assertAlmostEqual(second_ball["measurement_x"], 22.0)

    def test_confirmed_track_has_only_small_endpoint_tolerance(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            track_position_margin=0.02,
            smoothing_alpha=1.0,
        )
        tracker.update([(80, 0, 10, 100)], 0)
        tolerated = tracker.update([(101.5, 0, 10, 100)], 20)
        rejected = tracker.update([(104, 0, 10, 100)], 40)
        self.assertTrue(tolerated["measured"])
        self.assertFalse(rejected["measured"])

    def test_real_ball_can_use_calibrated_endpoint_margin(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            acquire_position_margin=0.08,
            track_position_margin=0.08,
            acquire_min_quality=80,
        )
        endpoint_ball = tracker.update([(106, -5, 12, 110)], 0)
        self.assertTrue(endpoint_ball["valid"])
        self.assertTrue(endpoint_ball["measured"])

    def test_endpoint_reflection_cannot_acquire_dynamic_pipe_track(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            acquire_position_margin=0.08,
            acquire_endpoint_inset=0.04,
        )
        reflection = tracker.update([(99, 0, 12, 120)], 0)
        real_ball = tracker.update([(80, 0, 12, 120)], 20)
        self.assertFalse(reflection["measured"])
        self.assertEqual(reflection["position_rejects"], 1)
        self.assertTrue(real_ball["measured"])

    def test_pipe_relative_fixture_masks_reject_both_endpoint_screws(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            acquire_endpoint_inset=0.0,
            acquire_position_margin=0.08,
            fixture_exclusions=(
                (0.02, -2.0, 4.0),
                (0.96, -3.0, 4.0),
            ),
        )
        left_screw = tracker.update([(2, -2, 3, 120)], 0)
        right_screw = tracker.update([(96, -3, 3, 120)], 20)
        left_ball = tracker.update([(9, -2, 7, 120)], 40)
        self.assertFalse(left_screw["measured"])
        self.assertEqual(left_screw["fixture_rejects"], 1)
        self.assertFalse(right_screw["measured"])
        self.assertEqual(right_screw["fixture_rejects"], 1)
        self.assertTrue(left_ball["measured"])

    def test_fixture_masks_follow_pipe_axis_rotation(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            fixture_exclusions=((0.02, -2.0, 4.0),),
        )
        tracker.set_axis((10, 10), (10, 110))
        screw = tracker.update([(12, 12, 3, 120)], 0)
        nearby_ball = tracker.update([(12, 20, 7, 120)], 20)
        self.assertFalse(screw["measured"])
        self.assertEqual(screw["fixture_rejects"], 1)
        self.assertTrue(nearby_ball["measured"])

    def test_confirmed_track_rejects_low_quality_reflection(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            track_min_quality=80,
        )
        tracker.update([(50, 0, 10, 120)], 0)
        reflection = tracker.update([(55, 0, 5, 20)], 20)
        self.assertFalse(reflection["measured"])
        self.assertTrue(reflection["coasting"])
        self.assertEqual(reflection["quality_rejects"], 1)

    def test_asymmetric_lateral_gate_rejects_background_below_pipe(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            max_axis_distance_px=30,
            max_below_axis_distance_px=10,
            confirm_frames=1,
        )
        below = tracker.update([(50, 15, 10, 100)], 0)
        above = tracker.update([(50, -20, 10, 100)], 20)
        self.assertFalse(below["valid"])
        self.assertEqual(below["lateral_rejects"], 1)
        self.assertTrue(above["valid"])

    def test_lost_track_requires_confirmation_again(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=2,
            coast_frames=1,
            memory_frames=4,
        )
        tracker.update([(40, 0, 10, 100)], 0)
        self.assertTrue(tracker.update([(42, 0, 10, 100)], 20)["valid"])
        self.assertTrue(tracker.update([], 40)["valid"])
        self.assertFalse(tracker.update([], 60)["valid"])
        self.assertFalse(tracker.update([(45, 0, 10, 100)], 80)["valid"])
        self.assertTrue(tracker.update([(47, 0, 10, 100)], 100)["valid"])

    def test_trusted_memory_reacquires_dim_ball_without_loosening_startup(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=2,
            coast_frames=1,
            memory_frames=4,
            acquire_min_quality=80,
            track_min_quality=60,
        )
        self.assertFalse(tracker.update([(40, 0, 10, 120)], 0)["valid"])
        self.assertTrue(tracker.update([(42, 0, 10, 120)], 20)["valid"])
        self.assertTrue(tracker.update([], 40)["valid"])
        self.assertFalse(tracker.update([], 60)["valid"])
        weak_first = tracker.update([(44, 0, 10, 70)], 80)
        weak_second = tracker.update([(46, 0, 10, 70)], 100)
        self.assertTrue(weak_first["measured"])
        self.assertFalse(weak_first["valid"])
        self.assertTrue(weak_second["valid"])

        fresh = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=2,
            acquire_min_quality=80,
            track_min_quality=60,
        )
        self.assertFalse(fresh.update([(40, 0, 10, 100)], 0)["valid"])
        low_quality_second = fresh.update([(42, 0, 10, 70)], 20)
        self.assertFalse(low_quality_second["measured"])
        self.assertEqual(low_quality_second["quality_rejects"], 1)

    def test_blob_geometry_filter(self):
        detector = LabBallDetector(
            640,
            480,
            (80, 125, 470, 85),
            ((0, 100, -20, 20, -15, 15),),
        )
        ball = detector._candidate_from_blob(
            FakeBlob(300, 155, 17, 16, 150, 0.85)
        )
        long_pipe_edge = detector._candidate_from_blob(
            FakeBlob(100, 150, 300, 5, 900, 0.05)
        )
        self.assertIsNotNone(ball)
        self.assertIsNone(long_pipe_edge)

    def test_ball_search_does_not_merge_ball_with_pipe_rail(self):
        class FakeImage:
            def __init__(self):
                self.kwargs = None

            def find_blobs(self, _thresholds, **kwargs):
                self.kwargs = kwargs
                return []

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (80, 125, 470, 85),
            ((0, 45, -20, 20, -15, 15),),
            merge_blobs=False,
        )
        detector.detect(image)
        self.assertFalse(image.kwargs["merge"])

    def test_native_circle_recovery_is_additive_during_acquisition(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return [FakeBlob(76, 133, 9, 10, 55, 0.9)]

            def find_circles(self, **kwargs):
                self.circle_kwargs = kwargs
                return [FakeCircle(488, 119, 14)]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_min_radius=11,
            circle_max_radius=18,
        )
        result = detector.detect(image)
        self.assertEqual(result["circle_count"], 1)
        self.assertEqual(len(result["candidates"]), 2)
        self.assertEqual(result["candidates"][-1][:3], (488.0, 119.0, 14.0))
        self.assertEqual(image.circle_kwargs["r_min"], 11)
        self.assertEqual(image.circle_kwargs["r_max"], 18)

    def test_weak_fixture_blob_does_not_suppress_circle_recovery(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return [FakeBlob(76, 133, 9, 10, 30, 0.5)]

            def find_circles(self, **_kwargs):
                return [FakeCircle(488, 145, 14)]

        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_trigger_min_quality=50,
        )
        result = detector.detect(FakeImage(), predicted_x=76, predicted_y=133)
        self.assertEqual(result["circle_count"], 1)

    def test_strong_off_axis_blob_does_not_suppress_circle_recovery(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return [FakeBlob(470, 90, 14, 12, 90, 0.9)]

            def find_circles(self, **_kwargs):
                return [FakeCircle(460, 145, 14)]

        detector = LabBallDetector(
            640,
            480,
            (45, 80, 540, 100),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_trigger_min_quality=40,
            circle_trigger_max_axis_distance_px=15,
        )
        detector.set_axis((70, 145), (520, 145))
        result = detector.detect(FakeImage(), predicted_x=480, predicted_y=145)
        self.assertEqual(result["circle_count"], 1)

    def test_vehicle_blob_centroid_bias_follows_pipe_axis(self):
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            blob_center_bias_along_axis_px=14,
            blob_center_bias_min_quality=40,
        )
        detector.set_axis((0, 0), (100, 0))
        strong = detector._candidate_from_blob(
            FakeBlob(200, 145, 14, 12, 90, 0.9)
        )
        weak = detector._candidate_from_blob(
            FakeBlob(80, 145, 12, 10, 30, 0.4)
        )
        self.assertAlmostEqual(strong[0], 193.0)
        self.assertAlmostEqual(weak[0], 86.0)

    def test_endpoint_hough_peak_moves_inward_but_keeps_raw_position(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                return [
                    FakeCircle(96, 0, 10),
                    FakeCircle(50, 0, 10),
                ]

        detector = LabBallDetector(
            100,
            40,
            (0, 0, 100, 30),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_endpoint_position=0.12,
            circle_endpoint_inward_bias_px=8,
        )
        detector.set_axis((0, 0), (100, 0))
        result = detector.detect(FakeImage())
        self.assertEqual(result["candidates"][0][0], 88.0)
        self.assertEqual(result["candidates"][0][5], 96.0)
        self.assertEqual(result["candidates"][1][0], 50.0)

    def test_endpoint_hough_bias_can_be_asymmetric(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                return [
                    FakeCircle(4, 0, 12),
                    FakeCircle(96, 0, 12),
                ]

        detector = LabBallDetector(
            100,
            40,
            (0, 0, 100, 30),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_endpoint_position=0.12,
            circle_left_endpoint_inward_bias_px=6,
            circle_right_endpoint_inward_bias_px=12,
        )
        detector.set_axis((0, 0), (100, 0))
        result = detector.detect(FakeImage())
        self.assertEqual(result["candidates"][0][0], 10.0)
        self.assertEqual(result["candidates"][1][0], 84.0)

    def test_fixture_rejection_uses_unshifted_candidate_coordinates(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            fixture_exclusions=((1.02, 0.0, 3.0),),
            track_position_margin=0.05,
        )
        shifted_screw = tracker.update(
            [(94, 0, 9, 100, 1, 102, 0)],
            0,
        )
        self.assertFalse(shifted_screw["measured"])
        self.assertEqual(shifted_screw["fixture_rejects"], 1)

    def test_strong_blob_can_cross_fixture_core_but_circle_cannot(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            fixture_exclusions=((0.95, 0.0, 5.0),),
            fixture_blob_override_quality=50,
        )
        screw = tracker.update([(95, 0, 8, 120, 1)], 0)
        self.assertFalse(screw["measured"])
        cold_ball = tracker.update([(95, 0, 8, 80, 0)], 20)
        self.assertFalse(cold_ball["measured"])
        tracker.update([(80, 0, 8, 80, 0)], 40)
        ball = tracker.update([(95, 0, 8, 80, 0)], 60)
        self.assertTrue(ball["measured"])

    def test_fixture_soft_halo_prefers_inner_ball_over_screw_edge(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            max_frame_jump_px=30,
            fixture_exclusions=((102.0 / 100.0, 0.0, 2.0),),
            fixture_soft_radius_scale=4.0,
            fixture_soft_penalty_per_px=3.0,
            track_position_margin=0.05,
            smoothing_alpha=1.0,
        )
        tracker.update([(90, 0, 8, 100, 0)], 0)
        state = tracker.update(
            [
                (97, 0, 8, 90, 1),
                (99, 0, 8, 95, 1),
            ],
            20,
        )
        self.assertEqual(state["measurement_x"], 97.0)

    def test_endpoint_snap_removes_stop_jitter_and_releases_with_hysteresis(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            smoothing_alpha=1.0,
            endpoint_snap_left_position=0.05,
            endpoint_snap_right_position=0.94,
            endpoint_snap_enter=0.10,
            endpoint_snap_exit=0.14,
            endpoint_snap_confirm_frames=2,
        )
        first = tracker.update([(8, 0, 9, 100)], 0)
        locked = tracker.update([(4, 0, 11, 100)], 20)
        jitter = tracker.update([(9, 0, 8, 100)], 40)
        released = tracker.update([(16, 0, 8, 100)], 60)
        self.assertEqual(first["measurement_x"], 8.0)
        self.assertEqual(locked["measurement_x"], 5.0)
        self.assertEqual(jitter["measurement_x"], 5.0)
        self.assertEqual(jitter["velocity_px_s"], 0.0)
        self.assertEqual(released["measurement_x"], 16.0)

    def test_broad_circle_acquisition_can_be_rate_limited(self):
        class FakeImage:
            def __init__(self):
                self.circle_calls = 0

            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                self.circle_calls += 1
                return []

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_acquire_interval_frames=4,
        )
        circle_counts = [detector.detect(image)["circle_count"] for _ in range(8)]
        self.assertEqual(circle_counts, [0] * 8)
        self.assertEqual(image.circle_calls, 2)

    def test_local_blob_miss_delays_broad_retry(self):
        class FakeImage:
            def __init__(self):
                self.blob_rois = []

            def find_blobs(self, _thresholds, **kwargs):
                self.blob_rois.append(tuple(kwargs["roi"]))
                return []

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            local_width=140,
            local_fallback_interval_misses=2,
        )

        first = detector.detect(
            image, predicted_x=300, predicted_y=145
        )
        second = detector.detect(
            image, predicted_x=300, predicted_y=145
        )

        self.assertFalse(first["fell_back"])
        self.assertTrue(second["fell_back"])
        self.assertEqual(
            [roi[2] for roi in image.blob_rois],
            [140, 500],
        )

    def test_blob_stride_is_configurable(self):
        class FakeImage:
            def __init__(self):
                self.calls = []

            def find_blobs(self, _thresholds, **kwargs):
                self.calls.append(kwargs)
                return []

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            blob_x_stride=3,
            blob_y_stride=4,
        )

        detector.detect(image)

        self.assertEqual(image.calls[0]["x_stride"], 3)
        self.assertEqual(image.calls[0]["y_stride"], 4)

    def test_broad_circle_acquisition_can_be_disabled(self):
        class FakeImage:
            def __init__(self):
                self.circle_calls = 0

            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                self.circle_calls += 1
                return [FakeCircle(300, 145, 12)]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_acquire_enabled=False,
        )
        circle_counts = [
            detector.detect(image)["circle_count"] for _ in range(8)
        ]
        self.assertEqual(circle_counts, [0] * 8)
        self.assertEqual(image.circle_calls, 0)

    def test_tracked_circle_recovery_can_be_rate_limited(self):
        class FakeImage:
            def __init__(self):
                self.circle_calls = 0

            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                self.circle_calls += 1
                return [FakeCircle(300, 145, 12)]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (80, 125, 470, 85),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_track_interval_frames=3,
        )
        counts = [
            detector.detect(
                image, predicted_x=300, predicted_y=145
            )["circle_count"]
            for _ in range(5)
        ]
        self.assertEqual(counts, [1, 0, 0, 1, 0])
        self.assertEqual(image.circle_calls, 2)

    def test_tracked_circle_recovery_can_be_endpoint_only(self):
        class FakeImage:
            def __init__(self):
                self.circle_calls = 0
                self.circle_rois = []

            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **kwargs):
                self.circle_calls += 1
                self.circle_rois.append(tuple(kwargs["roi"]))
                return [FakeCircle(120, 145, 12)]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            local_width=140,
            circle_enabled=True,
            circle_track_endpoint_only=True,
            circle_endpoint_position=0.12,
        )
        detector.set_axis((100, 145), (500, 145))

        middle = detector.detect(
            image, predicted_x=300, predicted_y=145
        )
        endpoint = detector.detect(
            image, predicted_x=120, predicted_y=145
        )

        self.assertEqual(middle["circle_count"], 0)
        self.assertEqual(endpoint["circle_count"], 1)
        self.assertEqual(image.circle_calls, 1)
        self.assertEqual(image.circle_rois[0][2], 140)

    def test_circle_color_filter_rejects_green_and_keeps_metal(self):
        class FakeImage:
            def find_blobs(self, _thresholds, **_kwargs):
                return []

            def find_circles(self, **_kwargs):
                return [
                    FakeCircle(200, 145, 12),
                    FakeCircle(490, 145, 13),
                ]

            def get_pixel(self, x, _y, rgbtuple=False):
                self.assert_rgbtuple = rgbtuple
                if x < 300:
                    return [15, 120, 50]
                return [110, 125, 118]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            circle_enabled=True,
            circle_color_filter=True,
            circle_min_neutral_samples=8,
        )
        result = detector.detect(image)
        self.assertTrue(image.assert_rgbtuple)
        self.assertEqual(result["circle_count"], 1)
        self.assertEqual(result["candidates"][0][:3], (490.0, 145.0, 13.0))

    def test_local_circle_runs_despite_unrelated_full_roi_blob(self):
        class FakeImage:
            def __init__(self):
                self.circle_roi = None

            def find_blobs(self, _thresholds, **kwargs):
                roi = kwargs["roi"]
                if roi[2] < 500:
                    return []
                return [FakeBlob(76, 133, 9, 10, 55, 0.9)]

            def find_circles(self, **kwargs):
                self.circle_roi = kwargs["roi"]
                return [FakeCircle(488, 145, 14)]

        image = FakeImage()
        detector = LabBallDetector(
            640,
            480,
            (45, 112, 500, 70),
            ((0, 85, -22, 22, -20, 20),),
            local_width=140,
            circle_enabled=True,
        )
        result = detector.detect(image, predicted_x=488, predicted_y=145)
        self.assertEqual(image.circle_roi[2], 140)
        self.assertEqual(result["circle_count"], 1)
        self.assertEqual(result["candidates"][-1][:3], (488.0, 145.0, 14.0))

    def test_local_search_roi_stays_inside_full_roi(self):
        full = (80, 125, 470, 85)
        self.assertEqual(
            local_search_roi(full, 100, 140, 640, 480),
            (80, 125, 140, 85),
        )
        self.assertEqual(
            local_search_roi(full, 520, 140, 640, 480),
            (410, 125, 140, 85),
        )
        self.assertEqual(
            local_search_roi(
                full,
                300,
                140,
                640,
                480,
                center_y=170,
                height=60,
            ),
            (230, 140, 140, 60),
        )

    def test_stm32_line_protocol(self):
        self.assertEqual(format_vision_line({"valid": False}), "none\n")
        self.assertEqual(
            format_vision_line(
                {"valid": True, "error_px": -12, "lateral_px": 3}
            ),
            "-12,3\n",
        )
        self.assertEqual(
            format_vision_line(
                {"valid": True, "error_px": -9, "lateral_px": 3},
                output_scale=4.0 / 3.0,
            ),
            "-12,4\n",
        )

    def test_tracker_exposes_raw_measurement_for_recording(self):
        tracker = BallTracker((0, 0), (100, 0), confirm_frames=1)
        state = tracker.update([(45, 2, 8, 90)], 0)
        self.assertEqual(state["measurement_x"], 45.0)
        self.assertEqual(state["measurement_y"], 2.0)
        coast = tracker.update([], 20)
        self.assertIsNone(coast["measurement_x"])
        self.assertIsNone(coast["measurement_y"])

    def test_axis_update_preserves_tracked_image_point(self):
        tracker = BallTracker(
            (0, 0),
            (100, 0),
            confirm_frames=1,
            smoothing_alpha=1.0,
        )
        tracker.update([(50, 5, 8, 100)], 0)
        before = tracker.predicted_point(0)
        tracker.set_axis((10, 10), (210, 10))
        after = tracker.predicted_point(0)
        self.assertAlmostEqual(before[0], after[0])
        self.assertAlmostEqual(before[1], after[1])
        self.assertAlmostEqual(tracker.axis_length, 200.0)

    def test_tracking_csv_has_stable_column_count(self):
        state = BallTracker(
            (0, 0), (100, 0), confirm_frames=1
        ).update([(45, 2, 8, 90)], 0)
        detection = {
            "raw_count": 2,
            "candidates": [(45, 2, 8, 90)],
            "search_roi": (0, 0, 100, 20),
            "fell_back": False,
        }
        row = tracking_row(
            0,
            0,
            10,
            10,
            50.0,
            1200,
            state,
            detection,
            4,
            (0, 0, 100, 20),
        )
        self.assertEqual(len(csv_header().strip().split(",")), len(CSV_FIELDS))
        self.assertEqual(len(row.strip().split(",")), len(CSV_FIELDS))

    def test_run_stats_summary(self):
        stats = RunStats()
        state = {
            "measured": True,
            "valid": True,
            "coasting": False,
        }
        stats.update(state, detect_ms=4, encoded_bytes=100)
        summary = stats.summary(20)
        self.assertIn("effective_fps=50.000", summary)
        self.assertIn("measured_ratio=1.000000", summary)

    def test_periodic_deadline_does_not_collapse_to_half_frame_rate(self):
        deadline = 0
        sent_at = []
        for now_ms in range(19, 1001, 19):
            due, deadline = periodic_due(now_ms, deadline, 20)
            if due:
                sent_at.append(now_ms)
        self.assertGreaterEqual(len(sent_at), 49)
        self.assertLessEqual(len(sent_at), 51)


if __name__ == "__main__":
    unittest.main()
