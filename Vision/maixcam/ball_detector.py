"""Fast LAB blob detector specialized for a steel ball in a green pipe."""

import math


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def clamp_roi(roi, frame_width, frame_height):
    x, y, w, h = [int(v) for v in roi]
    x = clamp(x, 0, max(0, frame_width - 1))
    y = clamp(y, 0, max(0, frame_height - 1))
    w = clamp(w, 1, frame_width - x)
    h = clamp(h, 1, frame_height - y)
    return (x, y, w, h)


def local_search_roi(
    full_roi,
    center_x,
    width,
    frame_width,
    frame_height,
    center_y=None,
    height=None,
):
    x, y, w, h = full_roi
    local_width = min(int(width), int(w))
    local_x = int(round(float(center_x) - local_width / 2.0))
    local_x = clamp(local_x, x, x + w - local_width)
    if center_y is None or height is None:
        local_y = y
        local_height = h
    else:
        local_height = min(int(height), int(h))
        local_y = int(round(float(center_y) - local_height / 2.0))
        local_y = clamp(local_y, y, y + h - local_height)
    return clamp_roi(
        (local_x, local_y, local_width, local_height),
        frame_width,
        frame_height,
    )


class LabBallDetector:
    """Use native ``find_blobs`` and reject non-ball geometry."""

    def __init__(
        self,
        frame_width,
        frame_height,
        full_roi,
        thresholds,
        local_width=140,
        local_height=76,
        local_fallback_interval_misses=1,
        min_width=4,
        max_width=38,
        min_height=4,
        max_height=30,
        min_pixels=10,
        max_pixels=650,
        min_density=0.10,
        max_aspect=4.5,
        merge_blobs=False,
        merge_margin=3,
        circle_enabled=False,
        circle_threshold=2000,
        circle_min_radius=10,
        circle_max_radius=20,
        circle_x_stride=2,
        circle_y_stride=2,
        circle_radius_step=2,
        circle_acquire_enabled=True,
        circle_acquire_interval_frames=1,
        circle_track_interval_frames=1,
        circle_track_endpoint_only=False,
        circle_x_margin=6,
        circle_y_margin=6,
        circle_r_margin=4,
        circle_color_filter=False,
        circle_max_chroma=40,
        circle_dark_value=75,
        circle_min_neutral_samples=8,
        circle_max_above_center=24,
        circle_max_below_center=9,
        blob_center_bias_along_axis_px=0,
        blob_center_bias_min_quality=0,
        circle_trigger_min_quality=0,
        circle_trigger_max_axis_distance_px=None,
        circle_endpoint_position=0.0,
        circle_endpoint_inward_bias_px=0.0,
        circle_left_endpoint_inward_bias_px=None,
        circle_right_endpoint_inward_bias_px=None,
    ):
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.full_roi = clamp_roi(
            full_roi, self.frame_width, self.frame_height
        )
        self.thresholds = [list(values) for values in thresholds]
        self.local_width = int(local_width)
        self.local_height = int(local_height)
        self.local_fallback_interval_misses = max(
            1, int(local_fallback_interval_misses)
        )
        self._local_miss_count = 0
        self.min_width = int(min_width)
        self.max_width = int(max_width)
        self.min_height = int(min_height)
        self.max_height = int(max_height)
        self.min_pixels = int(min_pixels)
        self.max_pixels = int(max_pixels)
        self.min_density = float(min_density)
        self.max_aspect = float(max_aspect)
        self.merge_blobs = bool(merge_blobs)
        self.merge_margin = int(merge_margin)
        self.circle_enabled = bool(circle_enabled)
        self.circle_threshold = int(circle_threshold)
        self.circle_min_radius = int(circle_min_radius)
        self.circle_max_radius = int(circle_max_radius)
        self.circle_x_stride = max(1, int(circle_x_stride))
        self.circle_y_stride = max(1, int(circle_y_stride))
        self.circle_radius_step = max(1, int(circle_radius_step))
        self.circle_acquire_enabled = bool(circle_acquire_enabled)
        self.circle_acquire_interval_frames = max(
            1, int(circle_acquire_interval_frames)
        )
        self.circle_track_interval_frames = max(
            1, int(circle_track_interval_frames)
        )
        self.circle_track_endpoint_only = bool(
            circle_track_endpoint_only
        )
        self._untracked_frame_count = 0
        self._tracked_frame_count = 0
        self.circle_x_margin = max(1, int(circle_x_margin))
        self.circle_y_margin = max(1, int(circle_y_margin))
        self.circle_r_margin = max(1, int(circle_r_margin))
        self.circle_color_filter = bool(circle_color_filter)
        self.circle_max_chroma = max(0, int(circle_max_chroma))
        self.circle_dark_value = max(0, int(circle_dark_value))
        self.circle_min_neutral_samples = max(
            1, int(circle_min_neutral_samples)
        )
        self.circle_max_above_center = max(
            0, int(circle_max_above_center)
        )
        self.circle_max_below_center = max(
            0, int(circle_max_below_center)
        )
        self.blob_center_bias_along_axis_px = max(
            0.0, float(blob_center_bias_along_axis_px)
        )
        self.blob_center_bias_min_quality = max(
            0.0, float(blob_center_bias_min_quality)
        )
        self.circle_trigger_min_quality = max(
            0.0, float(circle_trigger_min_quality)
        )
        self.circle_trigger_max_axis_distance_px = (
            None
            if circle_trigger_max_axis_distance_px is None
            else max(0.0, float(circle_trigger_max_axis_distance_px))
        )
        self.circle_endpoint_position = clamp(
            float(circle_endpoint_position), 0.0, 0.49
        )
        self.circle_endpoint_inward_bias_px = max(
            0.0, float(circle_endpoint_inward_bias_px)
        )
        self.circle_left_endpoint_inward_bias_px = max(
            0.0,
            float(
                self.circle_endpoint_inward_bias_px
                if circle_left_endpoint_inward_bias_px is None
                else circle_left_endpoint_inward_bias_px
            ),
        )
        self.circle_right_endpoint_inward_bias_px = max(
            0.0,
            float(
                self.circle_endpoint_inward_bias_px
                if circle_right_endpoint_inward_bias_px is None
                else circle_right_endpoint_inward_bias_px
            ),
        )
        self.axis_start = (0.0, 0.0)
        self.axis_unit = (1.0, 0.0)
        self.axis_length = 1.0

    def set_full_roi(self, full_roi):
        """Move the ball search window with the latest pipe pose."""
        self.full_roi = clamp_roi(
            full_roi, self.frame_width, self.frame_height
        )

    def set_axis(self, axis_start, axis_end):
        """Set the pipe direction used to correct the mounted-camera bias.

        The vehicle lighting leaves a compact neutral island on the ball's
        right-hand side.  ``find_blobs`` correctly finds that island, but its
        centroid is not the steel ball centre.  Expressing the calibration
        along the current pipe axis keeps the correction valid while the pipe
        tilts.
        """
        dx = float(axis_end[0]) - float(axis_start[0])
        dy = float(axis_end[1]) - float(axis_start[1])
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return
        self.axis_start = (
            float(axis_start[0]),
            float(axis_start[1]),
        )
        self.axis_unit = (dx / length, dy / length)
        self.axis_length = length

    def _candidate_is_near_axis(self, candidate):
        limit = self.circle_trigger_max_axis_distance_px
        if limit is None:
            return True
        dx = float(candidate[0]) - self.axis_start[0]
        dy = float(candidate[1]) - self.axis_start[1]
        lateral = self.axis_unit[0] * dy - self.axis_unit[1] * dx
        return abs(lateral) <= limit

    def _point_axis_position(self, x, y):
        dx = float(x) - self.axis_start[0]
        dy = float(y) - self.axis_start[1]
        along = dx * self.axis_unit[0] + dy * self.axis_unit[1]
        return along / max(1.0, self.axis_length)

    def _tracked_circle_region_allows(self, predicted_x, predicted_y):
        if not self.circle_track_endpoint_only:
            return True
        position = self._point_axis_position(predicted_x, predicted_y)
        endpoint = self.circle_endpoint_position
        return position <= endpoint or position >= 1.0 - endpoint

    def _find(self, img, roi):
        return img.find_blobs(
            self.thresholds,
            roi=list(roi),
            x_stride=2,
            y_stride=2,
            area_threshold=max(4, self.min_pixels),
            pixels_threshold=self.min_pixels,
            merge=self.merge_blobs,
            margin=self.merge_margin,
        )

    def _candidate_from_blob(self, blob):
        w = int(blob.w())
        h = int(blob.h())
        pixels = int(blob.pixels())
        if w < self.min_width or w > self.max_width:
            return None
        if h < self.min_height or h > self.max_height:
            return None
        if pixels < self.min_pixels or pixels > self.max_pixels:
            return None

        short_side = max(1.0, float(min(w, h)))
        long_side = float(max(w, h))
        aspect = long_side / short_side
        if aspect > self.max_aspect:
            return None

        area = max(1, w * h)
        density = float(pixels) / float(area)
        if density < self.min_density:
            return None

        try:
            roundness = float(blob.roundness())
        except Exception:
            roundness = min(1.0, short_side / long_side)

        shape_score = min(1.0, short_side / long_side)
        quality = (
            float(pixels)
            * (0.45 + 0.55 * shape_score)
            * (0.60 + 0.40 * max(0.0, roundness))
            * (0.65 + 0.35 * min(1.0, density))
        )
        radius = 0.25 * (float(w) + float(h))
        raw_center_x = float(blob.cxf())
        raw_center_y = float(blob.cyf())
        center_x = raw_center_x
        center_y = raw_center_y
        if (
            self.blob_center_bias_along_axis_px > 0.0
            and quality >= self.blob_center_bias_min_quality
        ):
            center_x -= (
                self.axis_unit[0] * self.blob_center_bias_along_axis_px
            )
            center_y -= (
                self.axis_unit[1] * self.blob_center_bias_along_axis_px
            )
        return (
            center_x,
            center_y,
            radius,
            quality,
            0,
            raw_center_x,
            raw_center_y,
        )

    def _convert(self, blobs):
        candidates = []
        accepted_blobs = []
        for blob in blobs:
            candidate = self._candidate_from_blob(blob)
            if candidate is None:
                continue
            candidates.append(candidate)
            accepted_blobs.append(blob)
        return candidates, accepted_blobs

    def _circle_looks_metallic(self, img, circle, roi):
        """Reject green-pipe Hough peaks using a tiny RGB sample set."""
        if not self.circle_color_filter:
            return True

        center_x = float(circle.x())
        center_y = float(circle.y())
        radius = float(circle.r())
        roi_center_y = float(roi[1]) + 0.5 * float(roi[3])
        if center_y < roi_center_y - self.circle_max_above_center:
            return False
        if center_y > roi_center_y + self.circle_max_below_center:
            return False

        offsets = (
            (0.0, 0.0),
            (0.45, 0.0),
            (-0.45, 0.0),
            (0.0, 0.45),
            (0.0, -0.45),
            (0.32, 0.32),
            (-0.32, 0.32),
            (0.32, -0.32),
            (-0.32, -0.32),
            (0.72, 0.0),
            (-0.72, 0.0),
            (0.0, 0.72),
            (0.0, -0.72),
        )
        neutral = 0
        sampled = 0
        for offset_x, offset_y in offsets:
            x = clamp(
                int(round(center_x + offset_x * radius)),
                0,
                self.frame_width - 1,
            )
            y = clamp(
                int(round(center_y + offset_y * radius)),
                0,
                self.frame_height - 1,
            )
            try:
                pixel = img.get_pixel(x, y, True)
            except Exception:
                return False
            if pixel is None or len(pixel) < 3:
                return False
            channels = [int(pixel[0]), int(pixel[1]), int(pixel[2])]
            highest = max(channels)
            chroma = highest - min(channels)
            if (
                chroma <= self.circle_max_chroma
                or highest < self.circle_dark_value
            ):
                neutral += 1
            sampled += 1
        required = min(self.circle_min_neutral_samples, sampled)
        return neutral >= required

    def _find_circle_candidates(self, img, roi):
        """Use MaixPy's native Hough implementation for reflective fixtures.

        On the car, a steel ball touching the right stop joins the aluminium
        bracket in the neutral LAB mask.  Its geometric circle remains
        distinct, while both fixed screws are below the configured radius.
        """
        if not self.circle_enabled:
            return [], []
        try:
            circles = img.find_circles(
                roi=list(roi),
                x_stride=self.circle_x_stride,
                y_stride=self.circle_y_stride,
                threshold=self.circle_threshold,
                x_margin=self.circle_x_margin,
                y_margin=self.circle_y_margin,
                r_margin=self.circle_r_margin,
                r_min=self.circle_min_radius,
                r_max=self.circle_max_radius,
                r_step=self.circle_radius_step,
            )
        except Exception:
            return [], []

        candidates = []
        accepted = []
        for circle in circles:
            radius = float(circle.r())
            if (
                radius < self.circle_min_radius
                or radius > self.circle_max_radius
            ):
                continue
            if not self._circle_looks_metallic(img, circle, roi):
                continue
            try:
                magnitude = float(circle.magnitude())
            except Exception:
                magnitude = float(self.circle_threshold)
            # Tracker quality is a relative acquisition gate.  Scale Hough
            # magnitude into the same useful 60--250 range as blob quality.
            quality = max(60.0, min(250.0, magnitude / 15.0))
            raw_x = float(circle.x())
            raw_y = float(circle.y())
            center_x = raw_x
            center_y = raw_y
            if (
                self.circle_endpoint_position > 0.0
                and (
                    self.circle_left_endpoint_inward_bias_px > 0.0
                    or self.circle_right_endpoint_inward_bias_px > 0.0
                )
                and self.axis_length > 0.0
            ):
                along = (
                    (raw_x - self.axis_start[0]) * self.axis_unit[0]
                    + (raw_y - self.axis_start[1]) * self.axis_unit[1]
                )
                position = along / self.axis_length
                if position <= self.circle_endpoint_position:
                    center_x += (
                        self.axis_unit[0]
                        * self.circle_left_endpoint_inward_bias_px
                    )
                    center_y += (
                        self.axis_unit[1]
                        * self.circle_left_endpoint_inward_bias_px
                    )
                elif position >= 1.0 - self.circle_endpoint_position:
                    center_x -= (
                        self.axis_unit[0]
                        * self.circle_right_endpoint_inward_bias_px
                    )
                    center_y -= (
                        self.axis_unit[1]
                        * self.circle_right_endpoint_inward_bias_px
                    )
            candidates.append(
                (
                    center_x,
                    center_y,
                    radius,
                    quality,
                    1,
                    raw_x,
                    raw_y,
                )
            )
            accepted.append(circle)
        return candidates, accepted

    def detect(self, img, predicted_x=None, predicted_y=None):
        """Return candidates, accepted blobs, search ROI and raw blob count.

        While locked, a small local ROI is searched first.  If it contains no
        accepted candidate, a full-pipe search is performed in the same frame.
        """
        if predicted_x is None:
            self._untracked_frame_count += 1
            self._tracked_frame_count = 0
            self._local_miss_count = 0
            search_roi = self.full_roi
            used_local = False
        else:
            self._untracked_frame_count = 0
            self._tracked_frame_count += 1
            search_roi = local_search_roi(
                self.full_roi,
                predicted_x,
                self.local_width,
                self.frame_width,
                self.frame_height,
                center_y=predicted_y,
                height=self.local_height,
            )
            used_local = search_roi != self.full_roi

        raw_blobs = self._find(img, search_roi)
        candidates, accepted = self._convert(raw_blobs)
        raw_count = len(raw_blobs)
        fell_back = False
        circle_candidates = []
        circles = []
        # Decide circle recovery from the high-value local result, before a
        # broad LAB fallback can add an unrelated endpoint screw candidate.
        # A weak fixed screw/printed mark must not suppress circle recovery.
        # Only a genuinely ball-like LAB blob is strong enough to skip the
        # more expensive Hough pass.
        if self.circle_trigger_min_quality <= 0.0:
            # Backwards-compatible mode: untracked acquisition always gets a
            # circle pass, even when a blob candidate exists.
            need_circle = predicted_x is None or not candidates
        else:
            has_strong_blob = any(
                float(candidate[3]) >= self.circle_trigger_min_quality
                and self._candidate_is_near_axis(candidate)
                for candidate in candidates
            )
            need_circle = not has_strong_blob

        circle_roi = search_roi
        if used_local and candidates:
            self._local_miss_count = 0
        elif used_local:
            self._local_miss_count += 1
            broad_retry_due = (
                self._local_miss_count
                % self.local_fallback_interval_misses
                == 0
            )
            if broad_retry_due:
                full_blobs = self._find(img, self.full_roi)
                candidates, accepted = self._convert(full_blobs)
                raw_count += len(full_blobs)
                search_roi = self.full_roi
                fell_back = True
                if candidates:
                    self._local_miss_count = 0

        # Circle acquisition is intentionally additive.  A fixed screw may
        # survive the LAB filter as a small candidate; that must not suppress
        # recovery of the larger ball beside it.
        tracked_circle_due = (
            (
                predicted_x is not None
                and self._tracked_circle_region_allows(
                    predicted_x, predicted_y
                )
                and (
                    (self._tracked_frame_count - 1)
                    % self.circle_track_interval_frames
                    == 0
                )
            )
        )
        acquisition_circle_due = (
            self.circle_acquire_enabled
            and (
                predicted_x is None
                and
                (self._untracked_frame_count - 1)
                % self.circle_acquire_interval_frames
                == 0
            )
        )
        if (
            self.circle_enabled
            and need_circle
            and (tracked_circle_due or acquisition_circle_due)
        ):
            # A tentative/confirmed track supplies a high-value local window.
            # Keep it even when the cheap blob pass fell back to the full ROI;
            # otherwise Hough scans the whole textured pipe on every frame.
            if predicted_x is None:
                circle_roi = self.full_roi
            circle_candidates, circles = self._find_circle_candidates(
                img, circle_roi
            )
            candidates.extend(circle_candidates)

        return {
            "candidates": candidates,
            "blobs": accepted,
            "circles": circles,
            "circle_count": len(circle_candidates),
            "search_roi": search_roi,
            "raw_count": raw_count,
            "fell_back": fell_back,
        }
