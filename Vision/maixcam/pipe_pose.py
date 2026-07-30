"""Lightweight pipe-pose estimation and dynamic ball-search geometry."""

import math


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def clamp_roi(roi, frame_width, frame_height):
    x, y, w, h = [int(round(value)) for value in roi]
    x = clamp(x, 0, max(0, int(frame_width) - 1))
    y = clamp(y, 0, max(0, int(frame_height) - 1))
    w = clamp(w, 1, int(frame_width) - x)
    h = clamp(h, 1, int(frame_height) - y)
    return (x, y, w, h)


def pose_from_corners(corners):
    """Return the major-axis pose of a minimum-area rectangle.

    MaixPy documents ``Blob.mini_corners()`` as four corners whose starting
    order may change.  A small 2-D principal-axis calculation avoids relying
    on that order or on firmware-specific rotation-angle conventions.
    """
    if corners is None or len(corners) < 4:
        return None
    points = [(float(point[0]), float(point[1])) for point in corners]
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    covariance_xx = 0.0
    covariance_yy = 0.0
    covariance_xy = 0.0
    for point_x, point_y in points:
        delta_x = point_x - center_x
        delta_y = point_y - center_y
        covariance_xx += delta_x * delta_x
        covariance_yy += delta_y * delta_y
        covariance_xy += delta_x * delta_y

    angle = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )
    unit_x = math.cos(angle)
    unit_y = math.sin(angle)
    if unit_x < 0.0:
        unit_x = -unit_x
        unit_y = -unit_y

    along = []
    across = []
    for point_x, point_y in points:
        delta_x = point_x - center_x
        delta_y = point_y - center_y
        along.append(delta_x * unit_x + delta_y * unit_y)
        across.append(-delta_x * unit_y + delta_y * unit_x)
    minimum_along = min(along)
    maximum_along = max(along)
    length = maximum_along - minimum_along
    width = max(across) - min(across)
    return {
        "center": (center_x, center_y),
        "unit": (unit_x, unit_y),
        "length": float(length),
        "width": float(width),
        "angle_rad": math.atan2(unit_y, unit_x),
        "start": (
            center_x + minimum_along * unit_x,
            center_y + minimum_along * unit_y,
        ),
        "end": (
            center_x + maximum_along * unit_x,
            center_y + maximum_along * unit_y,
        ),
    }


def inset_axis(pose, inset_px):
    inset = clamp(float(inset_px), 0.0, 0.45 * pose["length"])
    unit_x, unit_y = pose["unit"]
    start_x, start_y = pose["start"]
    end_x, end_y = pose["end"]
    return (
        (start_x + inset * unit_x, start_y + inset * unit_y),
        (end_x - inset * unit_x, end_y - inset * unit_y),
    )


def roi_from_axis(
    axis_start,
    axis_end,
    frame_width,
    frame_height,
    along_margin_px,
    lateral_margin_px,
):
    """Return an axis-aligned ROI enclosing a padded, rotated pipe axis."""
    start_x, start_y = axis_start
    end_x, end_y = axis_end
    vector_x = float(end_x - start_x)
    vector_y = float(end_y - start_y)
    length = math.hypot(vector_x, vector_y)
    if length <= 0.0:
        raise ValueError("axis start and end must be different")
    unit_x = vector_x / length
    unit_y = vector_y / length
    normal_x = -unit_y
    normal_y = unit_x
    along = max(0.0, float(along_margin_px))
    lateral = max(1.0, float(lateral_margin_px))
    padded_start = (
        start_x - along * unit_x,
        start_y - along * unit_y,
    )
    padded_end = (
        end_x + along * unit_x,
        end_y + along * unit_y,
    )
    corners = []
    for point_x, point_y in (padded_start, padded_end):
        corners.append(
            (point_x + lateral * normal_x, point_y + lateral * normal_y)
        )
        corners.append(
            (point_x - lateral * normal_x, point_y - lateral * normal_y)
        )
    minimum_x = min(point[0] for point in corners)
    maximum_x = max(point[0] for point in corners)
    minimum_y = min(point[1] for point in corners)
    maximum_y = max(point[1] for point in corners)
    return clamp_roi(
        (
            math.floor(minimum_x),
            math.floor(minimum_y),
            math.ceil(maximum_x) - math.floor(minimum_x) + 1,
            math.ceil(maximum_y) - math.floor(minimum_y) + 1,
        ),
        frame_width,
        frame_height,
    )


class GreenPipePoseDetector:
    """Track the elongated green pipe with native LAB blob extraction."""

    def __init__(
        self,
        frame_width,
        frame_height,
        search_roi,
        fallback_roi,
        fallback_axis_start,
        fallback_axis_end,
        thresholds,
        detect_interval_frames=2,
        min_length_px=260,
        max_length_px=580,
        min_width_px=8,
        max_width_px=100,
        min_aspect=4.0,
        min_pixels=500,
        merge_blobs=False,
        merge_margin=8,
        x_stride=4,
        y_stride=3,
        expected_center=None,
        max_center_distance_px=0,
        max_abs_angle_deg=0,
        fixed_axis_center=None,
        fixed_axis_length_px=0,
        pose_search_along_margin_px=12,
        pose_search_lateral_margin_px=36,
        smoothing_alpha=0.55,
        axis_inset_px=4,
        roi_along_margin_px=35,
        roi_lateral_margin_px=42,
        max_stale_frames=12,
        broad_retry_interval_updates=1,
        fixed_search_roi=False,
    ):
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.search_roi = clamp_roi(
            search_roi, self.frame_width, self.frame_height
        )
        self.fallback_roi = clamp_roi(
            fallback_roi, self.frame_width, self.frame_height
        )
        self.thresholds = [list(values) for values in thresholds]
        self.detect_interval_frames = max(1, int(detect_interval_frames))
        self.min_length_px = float(min_length_px)
        self.max_length_px = float(max_length_px)
        self.min_width_px = float(min_width_px)
        self.max_width_px = float(max_width_px)
        self.min_aspect = float(min_aspect)
        self.min_pixels = int(min_pixels)
        self.merge_blobs = bool(merge_blobs)
        self.merge_margin = int(merge_margin)
        self.x_stride = max(1, int(x_stride))
        self.y_stride = max(1, int(y_stride))
        self.expected_center = (
            None
            if expected_center is None
            else (float(expected_center[0]), float(expected_center[1]))
        )
        self.max_center_distance_px = max(
            0.0, float(max_center_distance_px)
        )
        self.max_abs_angle_rad = math.radians(
            max(0.0, float(max_abs_angle_deg))
        )
        self.fixed_axis_center = (
            None
            if fixed_axis_center is None
            else (
                float(fixed_axis_center[0]),
                float(fixed_axis_center[1]),
            )
        )
        self.fixed_axis_length_px = max(0.0, float(fixed_axis_length_px))
        self.pose_search_along_margin_px = max(
            1.0, float(pose_search_along_margin_px)
        )
        self.pose_search_lateral_margin_px = max(
            1.0, float(pose_search_lateral_margin_px)
        )
        self.smoothing_alpha = clamp(float(smoothing_alpha), 0.0, 1.0)
        self.axis_inset_px = max(0.0, float(axis_inset_px))
        self.roi_along_margin_px = max(0.0, float(roi_along_margin_px))
        self.roi_lateral_margin_px = max(
            1.0, float(roi_lateral_margin_px)
        )
        self.max_stale_frames = max(0, int(max_stale_frames))
        self.broad_retry_interval_updates = max(
            1, int(broad_retry_interval_updates)
        )
        self.fixed_search_roi = bool(fixed_search_roi)
        self.axis_start = tuple(float(v) for v in fallback_axis_start)
        self.axis_end = tuple(float(v) for v in fallback_axis_end)
        self.ball_roi = self.fallback_roi
        self.has_measurement = False
        self.age_frames = self.max_stale_frames + 1
        self.last_score = 0.0
        self.last_length = 0.0
        self.last_width = 0.0
        self.scheduled_misses = 0

    def _find(self, img, roi):
        return img.find_blobs(
            self.thresholds,
            roi=list(roi),
            x_stride=self.x_stride,
            y_stride=self.y_stride,
            area_threshold=self.min_pixels,
            pixels_threshold=self.min_pixels,
            merge=self.merge_blobs,
            margin=self.merge_margin,
        )

    def _adaptive_search_roi(self):
        if self.fixed_search_roi or not self.has_measurement:
            return self.search_roi
        return roi_from_axis(
            self.axis_start,
            self.axis_end,
            self.frame_width,
            self.frame_height,
            self.pose_search_along_margin_px,
            self.pose_search_lateral_margin_px,
        )

    def _candidates(self, blobs):
        candidates = []
        for blob in blobs:
            candidate = self._candidate(blob)
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _candidate(self, blob):
        try:
            pose = pose_from_corners(blob.mini_corners())
            pixels = int(blob.pixels())
        except Exception:
            return None
        if pose is None:
            return None
        length = pose["length"]
        width = pose["width"]
        aspect = length / max(1.0, width)
        if length < self.min_length_px or length > self.max_length_px:
            return None
        if width < self.min_width_px or width > self.max_width_px:
            return None
        if aspect < self.min_aspect or pixels < self.min_pixels:
            return None
        if (
            self.max_abs_angle_rad > 0.0
            and abs(pose["angle_rad"]) > self.max_abs_angle_rad
        ):
            return None
        center_distance = 0.0
        if self.expected_center is not None:
            center_distance = math.hypot(
                pose["center"][0] - self.expected_center[0],
                pose["center"][1] - self.expected_center[1],
            )
            if (
                self.max_center_distance_px > 0.0
                and center_distance > self.max_center_distance_px
            ):
                return None
        # Prefer a long, narrow component near the known chassis pivot.
        score = (
            length * length / max(12.0, width)
            + 0.15 * float(pixels)
            - 2.0 * center_distance
        )
        return score, pose

    def _apply_pose(self, pose, score):
        if (
            self.fixed_axis_center is not None
            and self.fixed_axis_length_px > 0.0
        ):
            center_x, center_y = self.fixed_axis_center
            unit_x, unit_y = pose["unit"]
            half_length = 0.5 * self.fixed_axis_length_px
            measured_start = (
                center_x - half_length * unit_x,
                center_y - half_length * unit_y,
            )
            measured_end = (
                center_x + half_length * unit_x,
                center_y + half_length * unit_y,
            )
        else:
            measured_start, measured_end = inset_axis(
                pose, self.axis_inset_px
            )
        if not self.has_measurement:
            self.axis_start = measured_start
            self.axis_end = measured_end
        else:
            alpha = self.smoothing_alpha
            self.axis_start = (
                self.axis_start[0]
                + alpha * (measured_start[0] - self.axis_start[0]),
                self.axis_start[1]
                + alpha * (measured_start[1] - self.axis_start[1]),
            )
            self.axis_end = (
                self.axis_end[0]
                + alpha * (measured_end[0] - self.axis_end[0]),
                self.axis_end[1]
                + alpha * (measured_end[1] - self.axis_end[1]),
            )
        self.ball_roi = roi_from_axis(
            self.axis_start,
            self.axis_end,
            self.frame_width,
            self.frame_height,
            self.roi_along_margin_px,
            self.roi_lateral_margin_px,
        )
        self.has_measurement = True
        self.age_frames = 0
        self.last_score = float(score)
        self.last_length = float(pose["length"])
        self.last_width = float(pose["width"])

    def update(self, img, frame_id):
        measured = False
        raw_blob_count = 0
        fell_back = False
        used_search_roi = self._adaptive_search_roi()
        scheduled = (
            not self.has_measurement
            or int(frame_id) % self.detect_interval_frames == 0
        )
        if scheduled:
            blobs = self._find(img, used_search_roi)
            raw_blob_count = len(blobs)
            candidates = self._candidates(blobs)
            if candidates:
                self.scheduled_misses = 0
            else:
                self.scheduled_misses += 1
            broad_retry_due = (
                not self.has_measurement
                or (
                    self.scheduled_misses
                    % self.broad_retry_interval_updates
                    == 0
                )
            )
            if (
                not candidates
                and used_search_roi != self.search_roi
                and broad_retry_due
            ):
                broad_blobs = self._find(img, self.search_roi)
                raw_blob_count += len(broad_blobs)
                candidates = self._candidates(broad_blobs)
                used_search_roi = self.search_roi
                fell_back = True
            if candidates:
                candidates.sort(key=lambda item: item[0], reverse=True)
                score, pose = candidates[0]
                self._apply_pose(pose, score)
                self.scheduled_misses = 0
                measured = True

        if not measured:
            self.age_frames += 1
        return {
            "axis_start": self.axis_start,
            "axis_end": self.axis_end,
            "ball_roi": self.ball_roi,
            "measured": measured,
            "valid": bool(
                self.has_measurement
                and self.age_frames <= self.max_stale_frames
            ),
            "age_frames": int(self.age_frames),
            "raw_blob_count": int(raw_blob_count),
            "search_roi": used_search_roi,
            "fell_back": bool(fell_back),
            "score": float(self.last_score),
            "length": float(self.last_length),
            "width": float(self.last_width),
        }


class TapeEndpointPipePoseDetector:
    """Estimate pipe pose from a fixed left end and the moving right tape.

    The competition mechanism has a camera-fixed left black tape marker.  The
    right tape marker follows the pipe rotation.  In the mounted view, direct
    black segmentation joins that tape to the dark motor/cable background, so
    the stable observable is the green-to-black boundary at its inner edge.
    One small endpoint search supplies both pipe angle and the right endpoint.
    """

    def __init__(
        self,
        frame_width,
        frame_height,
        right_search_roi,
        fallback_roi,
        fixed_left_endpoint,
        fallback_right_endpoint,
        thresholds,
        detect_interval_frames=3,
        min_width_px=7,
        max_width_px=32,
        min_height_px=18,
        max_height_px=48,
        min_pixels=45,
        x_stride=3,
        y_stride=3,
        expected_right_x=None,
        max_right_x_distance_px=0,
        min_axis_length_px=300,
        max_axis_length_px=420,
        max_abs_angle_deg=8,
        endpoint_from_blob_right_edge=False,
        endpoint_x_offset_px=0,
        smoothing_alpha=0.70,
        roi_along_margin_px=0,
        roi_lateral_margin_px=12,
        max_stale_frames=12,
    ):
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.right_search_roi = clamp_roi(
            right_search_roi,
            self.frame_width,
            self.frame_height,
        )
        self.fallback_roi = clamp_roi(
            fallback_roi,
            self.frame_width,
            self.frame_height,
        )
        self.fixed_left_endpoint = tuple(
            float(value) for value in fixed_left_endpoint
        )
        self.fallback_right_endpoint = tuple(
            float(value) for value in fallback_right_endpoint
        )
        self.thresholds = [list(values) for values in thresholds]
        self.detect_interval_frames = max(1, int(detect_interval_frames))
        self.min_width_px = max(1, int(min_width_px))
        self.max_width_px = max(self.min_width_px, int(max_width_px))
        self.min_height_px = max(1, int(min_height_px))
        self.max_height_px = max(self.min_height_px, int(max_height_px))
        self.min_pixels = max(1, int(min_pixels))
        self.x_stride = max(1, int(x_stride))
        self.y_stride = max(1, int(y_stride))
        self.expected_right_x = (
            None
            if expected_right_x is None
            else float(expected_right_x)
        )
        self.max_right_x_distance_px = max(
            0.0, float(max_right_x_distance_px)
        )
        self.min_axis_length_px = max(1.0, float(min_axis_length_px))
        self.max_axis_length_px = max(
            self.min_axis_length_px,
            float(max_axis_length_px),
        )
        self.max_abs_angle_rad = math.radians(
            max(0.0, float(max_abs_angle_deg))
        )
        self.endpoint_from_blob_right_edge = bool(
            endpoint_from_blob_right_edge
        )
        self.endpoint_x_offset_px = float(endpoint_x_offset_px)
        self.smoothing_alpha = clamp(float(smoothing_alpha), 0.0, 1.0)
        self.roi_along_margin_px = max(0.0, float(roi_along_margin_px))
        self.roi_lateral_margin_px = max(
            1.0, float(roi_lateral_margin_px)
        )
        self.max_stale_frames = max(0, int(max_stale_frames))

        self.axis_start = self.fixed_left_endpoint
        self.axis_end = self.fallback_right_endpoint
        self.ball_roi = self.fallback_roi
        self.has_measurement = False
        self.age_frames = self.max_stale_frames + 1
        self.last_score = 0.0
        self.last_length = math.hypot(
            self.axis_end[0] - self.axis_start[0],
            self.axis_end[1] - self.axis_start[1],
        )
        self.last_width = 0.0

    def _find(self, img):
        return img.find_blobs(
            self.thresholds,
            roi=list(self.right_search_roi),
            x_stride=self.x_stride,
            y_stride=self.y_stride,
            area_threshold=self.min_pixels,
            pixels_threshold=self.min_pixels,
            merge=False,
            margin=0,
        )

    def _candidate(self, blob):
        try:
            left = int(blob.x())
            width = int(blob.w())
            height = int(blob.h())
            pixels = int(blob.pixels())
            blob_center_x = float(blob.cx())
            center_y = float(blob.cy())
        except Exception:
            return None
        if width < self.min_width_px or width > self.max_width_px:
            return None
        if height < self.min_height_px or height > self.max_height_px:
            return None
        if pixels < self.min_pixels:
            return None
        center_x = (
            float(left + width - 1) + self.endpoint_x_offset_px
            if self.endpoint_from_blob_right_edge
            else blob_center_x
        )
        if (
            self.expected_right_x is not None
            and self.max_right_x_distance_px > 0.0
            and abs(center_x - self.expected_right_x)
            > self.max_right_x_distance_px
        ):
            return None

        delta_x = center_x - self.fixed_left_endpoint[0]
        delta_y = center_y - self.fixed_left_endpoint[1]
        length = math.hypot(delta_x, delta_y)
        if (
            length < self.min_axis_length_px
            or length > self.max_axis_length_px
        ):
            return None
        angle = math.atan2(delta_y, delta_x)
        if (
            self.max_abs_angle_rad > 0.0
            and abs(angle) > self.max_abs_angle_rad
        ):
            return None

        x_error = (
            0.0
            if self.expected_right_x is None
            else abs(center_x - self.expected_right_x)
        )
        score = (
            float(pixels)
            + 2.0 * float(height)
            + float(width)
            - 3.0 * x_error
        )
        return score, (center_x, center_y), length, min(width, height)

    def update(self, img, frame_id):
        measured = False
        raw_blob_count = 0
        scheduled = (
            not self.has_measurement
            or int(frame_id) % self.detect_interval_frames == 0
        )
        if scheduled:
            blobs = self._find(img)
            raw_blob_count = len(blobs)
            candidates = []
            for blob in blobs:
                candidate = self._candidate(blob)
                if candidate is not None:
                    candidates.append(candidate)
            if candidates:
                candidates.sort(key=lambda item: item[0], reverse=True)
                score, right_endpoint, length, marker_width = candidates[0]
                if self.has_measurement:
                    alpha = self.smoothing_alpha
                    self.axis_end = (
                        self.axis_end[0]
                        + alpha * (right_endpoint[0] - self.axis_end[0]),
                        self.axis_end[1]
                        + alpha * (right_endpoint[1] - self.axis_end[1]),
                    )
                else:
                    self.axis_end = right_endpoint
                self.axis_start = self.fixed_left_endpoint
                self.ball_roi = roi_from_axis(
                    self.axis_start,
                    self.axis_end,
                    self.frame_width,
                    self.frame_height,
                    self.roi_along_margin_px,
                    self.roi_lateral_margin_px,
                )
                self.has_measurement = True
                self.age_frames = 0
                self.last_score = float(score)
                self.last_length = float(length)
                self.last_width = float(marker_width)
                measured = True

        if not measured:
            self.age_frames += 1
        return {
            "axis_start": self.axis_start,
            "axis_end": self.axis_end,
            "ball_roi": self.ball_roi,
            "measured": measured,
            "valid": bool(
                self.has_measurement
                and self.age_frames <= self.max_stale_frames
            ),
            "age_frames": int(self.age_frames),
            "raw_blob_count": int(raw_blob_count),
            "search_roi": self.right_search_roi,
            "fell_back": False,
            "score": float(self.last_score),
            "length": float(self.last_length),
            "width": float(self.last_width),
        }
