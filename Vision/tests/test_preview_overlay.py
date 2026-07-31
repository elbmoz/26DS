import sys
import unittest
from pathlib import Path

import numpy


WINDOWS_DIR = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(WINDOWS_DIR))

from preview_overlay import (
    STATUS_BAR_HEIGHT,
    _q9_overlay_lines,
    build_preview_frame,
)


class PreviewOverlayTests(unittest.TestCase):
    def test_q9_overlay_lines_show_position_angles_and_status(self):
        lines = _q9_overlay_lines(
            {
                "motor_position": 4897,
                "angle_x_deg": -12.3,
                "angle_y_deg": 4.8,
                "angle_z_deg": 90.6,
                "imu_valid": 1,
                "position_valid": 0,
                "position_status": 2,
                "position_updates": 84,
                "move_direction": -1,
                "move_status": 3,
            }
        )
        self.assertEqual(
            lines,
            (
                "Q9 P:4897",
                "X:-12.3 Y:4.8 Z:90.6",
                "IMU:V POS:X RX:2 N:84",
                "DIR:-1 MOVE:3",
            ),
        )

    def test_status_footer_keeps_the_complete_video_visible(self):
        frame = numpy.full((24, 32, 3), (12, 34, 56), dtype=numpy.uint8)

        preview = build_preview_frame(
            frame,
            tracking=None,
            status=None,
            recording=False,
            sync_info={"matched": False},
        )

        self.assertEqual(
            preview.shape,
            (24 + STATUS_BAR_HEIGHT, 32, 3),
        )
        numpy.testing.assert_array_equal(preview[:24], frame)
        self.assertTrue(
            numpy.any(preview[24:] != frame[0, 0])
        )

    def test_annotations_do_not_mutate_the_decoded_source_frame(self):
        frame = numpy.zeros((96, 128, 3), dtype=numpy.uint8)
        original = frame.copy()
        status = {
            "camera_size": [640, 480],
            "roi": [40, 50, 500, 120],
            "axis_start": [60, 100],
            "axis_end": [560, 100],
            "config": {"target_position": 0.5},
        }

        build_preview_frame(
            frame,
            tracking=None,
            status=status,
            recording=True,
            sync_info={"matched": True},
        )

        numpy.testing.assert_array_equal(frame, original)

    def test_synchronized_frame_uses_tracking_axis_not_status_axis(self):
        frame = numpy.zeros((96, 128, 3), dtype=numpy.uint8)
        status = {
            "camera_size": [128, 96],
            "roi": [10, 70, 100, 20],
            "axis_start": [10, 80],
            "axis_end": [118, 80],
            "config": {"target_position": 0.5},
        }
        tracking = {
            "valid": False,
            "axis_x0": 10,
            "axis_y0": 20,
            "axis_x1": 118,
            "axis_y1": 20,
            "roi_x": 5,
            "roi_y": 8,
            "roi_w": 120,
            "roi_h": 30,
        }
        preview = build_preview_frame(
            frame,
            tracking=tracking,
            status=status,
            recording=False,
            sync_info={"matched": True},
        )
        self.assertTrue(numpy.any(preview[20, :, 1] >= 200))
        self.assertFalse(numpy.any(preview[80, :, 1] >= 200))

    def test_synchronized_frame_draws_effective_roi_quadrilateral(self):
        frame = numpy.zeros((96, 128, 3), dtype=numpy.uint8)
        status = {
            "camera_size": [128, 96],
            "config": {"target_position": 0.5},
        }
        tracking = {
            "valid": False,
            "axis_x0": 10,
            "axis_y0": 60,
            "axis_x1": 118,
            "axis_y1": 60,
            "roi_x": 0,
            "roi_y": 0,
            "roi_w": 128,
            "roi_h": 50,
            "roi_quad": [
                [5, 8],
                [123, 18],
                [118, 38],
                [5, 28],
            ],
        }

        preview = build_preview_frame(
            frame,
            tracking=tracking,
            status=status,
            recording=False,
            sync_info={"matched": True},
        )

        self.assertGreaterEqual(preview[13, 64, 0], 200)


if __name__ == "__main__":
    unittest.main()
