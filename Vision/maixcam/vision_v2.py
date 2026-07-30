"""Compact steel-ball recognition pipeline.

V2 deliberately models only what the mechanism guarantees:

* the left end of the pipe is fixed in the image;
* the right end moves mostly vertically and is marked by green pipe;
* the ball is a dark, nearly neutral blob close to the pipe axis;
* motion is continuous for a few frames.

All state is kept in small typed objects.  Dictionaries are created only by
the two compatibility adapters at the bottom of this file.
"""

import math


POSE_INTERVAL_FRAMES = 4
POSE_COAST_FRAMES = 12
FILTER_ALPHA = 0.72
FILTER_BETA = 0.14
LATERAL_ALPHA = 0.55
CONFIRM_FRAMES = 2
COAST_FRAMES = 2
MEMORY_FRAMES = 6


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def clamp_roi(roi, width, height):
    x, y, w, h = [int(round(value)) for value in roi]
    x = clamp(x, 0, max(0, width - 1))
    y = clamp(y, 0, max(0, height - 1))
    w = clamp(w, 1, width - x)
    h = clamp(h, 1, height - y)
    return (x, y, w, h)


class Point:
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def tuple(self):
        return (self.x, self.y)


class Axis:
    __slots__ = ("start", "end", "ux", "uy", "length")

    def __init__(self, start, end):
        self.start = Point(start[0], start[1])
        self.end = Point(end[0], end[1])
        dx = self.end.x - self.start.x
        dy = self.end.y - self.start.y
        self.length = math.hypot(dx, dy)
        if self.length <= 1.0:
            raise ValueError("pipe axis is too short")
        self.ux = dx / self.length
        self.uy = dy / self.length

    def project(self, point):
        dx = point.x - self.start.x
        dy = point.y - self.start.y
        along = dx * self.ux + dy * self.uy
        lateral = self.ux * dy - self.uy * dx
        return along, lateral

    def point(self, along, lateral=0.0):
        return Point(
            self.start.x + along * self.ux - lateral * self.uy,
            self.start.y + along * self.uy + lateral * self.ux,
        )

    def roi(self, margin, frame_width, frame_height):
        left = min(self.start.x, self.end.x) - margin
        top = min(self.start.y, self.end.y) - margin
        right = max(self.start.x, self.end.x) + margin
        bottom = max(self.start.y, self.end.y) + margin
        return clamp_roi(
            (left, top, right - left, bottom - top),
            frame_width,
            frame_height,
        )

    def quad(self, margin):
        nx = -self.uy * margin
        ny = self.ux * margin
        return (
            (self.start.x + nx, self.start.y + ny),
            (self.end.x + nx, self.end.y + ny),
            (self.end.x - nx, self.end.y - ny),
            (self.start.x - nx, self.start.y - ny),
        )


class VisionV2Config:
    """Only installation calibration belongs here; policy stays in V2."""

    __slots__ = (
        "frame_width",
        "frame_height",
        "left_endpoint",
        "right_endpoint",
        "right_search_roi",
        "pipe_thresholds",
        "ball_thresholds",
        "ball_diameter_px",
        "target_position",
    )

    def __init__(
        self,
        frame_width,
        frame_height,
        left_endpoint,
        right_endpoint,
        right_search_roi,
        pipe_thresholds,
        ball_thresholds,
        ball_diameter_px,
        target_position=0.5,
    ):
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.left_endpoint = tuple(left_endpoint)
        self.right_endpoint = tuple(right_endpoint)
        self.right_search_roi = clamp_roi(
            right_search_roi, self.frame_width, self.frame_height
        )
        self.pipe_thresholds = [list(values) for values in pipe_thresholds]
        self.ball_thresholds = [list(values) for values in ball_thresholds]
        self.ball_diameter_px = max(6.0, float(ball_diameter_px))
        self.target_position = clamp(float(target_position), 0.0, 1.0)


class Candidate:
    __slots__ = ("point", "radius", "quality", "along", "lateral", "blob")

    def __init__(self, point, radius, quality, along, lateral, blob):
        self.point = point
        self.radius = float(radius)
        self.quality = float(quality)
        self.along = float(along)
        self.lateral = float(lateral)
        self.blob = blob

    def legacy_tuple(self):
        return (
            self.point.x,
            self.point.y,
            self.radius,
            self.quality,
        )


class PipeState:
    __slots__ = (
        "axis",
        "roi",
        "corridor",
        "measured",
        "valid",
        "acquired",
        "age",
        "raw_count",
        "score",
    )

    def __init__(self, axis, roi, corridor):
        self.axis = axis
        self.roi = roi
        self.corridor = float(corridor)
        self.measured = False
        self.valid = False
        self.acquired = False
        self.age = 0
        self.raw_count = 0
        self.score = 0.0


class TrackState:
    __slots__ = (
        "valid",
        "measured",
        "coasting",
        "point",
        "measurement",
        "radius",
        "position",
        "position_px",
        "error_px",
        "lateral",
        "velocity",
        "quality",
        "hits",
        "misses",
    )

    def __init__(
        self,
        valid,
        measured,
        point,
        measurement,
        radius,
        position,
        position_px,
        error_px,
        lateral,
        velocity,
        quality,
        hits,
        misses,
    ):
        self.valid = bool(valid)
        self.measured = bool(measured)
        self.coasting = bool(valid and not measured)
        self.point = point
        self.measurement = measurement
        self.radius = float(radius)
        self.position = float(position)
        self.position_px = float(position_px)
        self.error_px = int(round(error_px))
        self.lateral = float(lateral)
        self.velocity = float(velocity)
        self.quality = float(quality)
        self.hits = int(hits)
        self.misses = int(misses)


class TrackerV2:
    __slots__ = (
        "axis",
        "target",
        "position_px",
        "lateral_px",
        "velocity_px_s",
        "radius",
        "last_ms",
        "hits",
        "misses",
        "confirmed",
        "quality",
    )

    def __init__(self, axis, target):
        self.axis = axis
        self.target = float(target)
        self.position_px = None
        self.lateral_px = 0.0
        self.velocity_px_s = 0.0
        self.radius = 0.0
        self.last_ms = None
        self.hits = 0
        self.misses = 0
        self.confirmed = False
        self.quality = 0.0

    def set_axis(self, axis):
        old_point = self.predicted_point(self.last_ms)
        old_length = self.axis.length
        self.axis = axis
        if old_point is None:
            return
        self.position_px, self.lateral_px = axis.project(old_point)
        self.position_px = clamp(self.position_px, 0.0, axis.length)
        self.velocity_px_s *= axis.length / old_length

    def _dt(self, now_ms):
        if self.last_ms is None or now_ms is None or now_ms <= self.last_ms:
            return 0.0
        return min(0.2, (int(now_ms) - int(self.last_ms)) / 1000.0)

    def predicted_position(self, now_ms):
        if self.position_px is None:
            return None
        return clamp(
            self.position_px + self.velocity_px_s * self._dt(now_ms),
            0.0,
            self.axis.length,
        )

    def predicted_point(self, now_ms):
        position = self.predicted_position(now_ms)
        if position is None:
            return None
        return self.axis.point(position, self.lateral_px)

    def _select(self, candidates, now_ms, diameter):
        if not candidates:
            return None
        predicted = self.predicted_position(now_ms)
        if predicted is None:
            return max(
                candidates,
                key=lambda item: item.quality - 2.0 * abs(item.lateral),
            )

        maximum_jump = 4.0 * diameter
        nearby = [
            item
            for item in candidates
            if abs(item.along - predicted) <= maximum_jump
        ]
        if not nearby:
            return None
        return min(
            nearby,
            key=lambda item: (
                abs(item.along - predicted)
                + 0.6 * abs(item.lateral)
                - 0.02 * item.quality
            ),
        )

    def _state(self, valid, measured, measurement=None):
        if self.position_px is None:
            return TrackState(
                False,
                False,
                Point(0, 0),
                None,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                self.misses,
            )
        point = self.axis.point(self.position_px, self.lateral_px)
        position = self.position_px / self.axis.length
        return TrackState(
            valid,
            measured,
            point,
            measurement,
            self.radius,
            position,
            self.position_px,
            self.position_px - self.target * self.axis.length,
            self.lateral_px,
            self.velocity_px_s,
            self.quality,
            self.hits,
            self.misses,
        )

    def update(self, candidates, now_ms, diameter):
        selected = self._select(candidates, now_ms, diameter)
        if selected is None:
            self.hits = 0
            self.misses += 1
            predicted = self.predicted_position(now_ms)
            if predicted is not None:
                self.position_px = predicted
                self.last_ms = int(now_ms)
            if self.confirmed and self.misses <= COAST_FRAMES:
                self.quality *= 0.65
                return self._state(True, False)
            self.confirmed = False
            if self.misses > MEMORY_FRAMES:
                self.position_px = None
                self.velocity_px_s = 0.0
                self.radius = 0.0
            return self._state(False, False)

        dt = self._dt(now_ms)
        measurement = selected
        if self.position_px is None:
            self.position_px = selected.along
            self.lateral_px = selected.lateral
            self.radius = selected.radius
        else:
            predicted = self.predicted_position(now_ms)
            residual = selected.along - predicted
            self.position_px = predicted + FILTER_ALPHA * residual
            if dt >= 0.001:
                self.velocity_px_s += FILTER_BETA * residual / dt
            self.lateral_px += LATERAL_ALPHA * (
                selected.lateral - self.lateral_px
            )
            self.radius += FILTER_ALPHA * (selected.radius - self.radius)
        self.position_px = clamp(self.position_px, 0.0, self.axis.length)
        self.last_ms = int(now_ms)
        self.hits += 1
        self.misses = 0
        self.confirmed = self.confirmed or self.hits >= CONFIRM_FRAMES
        self.quality = selected.quality
        return self._state(self.confirmed, True, measurement)


class DetectionResult:
    __slots__ = (
        "candidates",
        "blobs",
        "search_roi",
        "full_roi",
        "used_local",
        "fell_back",
        "raw_count",
        "pipe",
    )

    def __init__(
        self,
        candidates,
        search_roi,
        full_roi,
        used_local,
        fell_back,
        raw_count,
        pipe,
    ):
        self.candidates = candidates
        self.blobs = [item.blob for item in candidates]
        self.search_roi = search_roi
        self.full_roi = full_roi
        self.used_local = bool(used_local)
        self.fell_back = bool(fell_back)
        self.raw_count = int(raw_count)
        self.pipe = pipe


class BallVisionV2:
    """One object owns pose, detection and tracking for each frame."""

    __slots__ = ("config", "pipe", "tracker", "frame_id")

    def __init__(self, config):
        self.config = config
        axis = Axis(config.left_endpoint, config.right_endpoint)
        roi = axis.roi(
            1.5 * config.ball_diameter_px,
            config.frame_width,
            config.frame_height,
        )
        self.pipe = PipeState(
            axis, roi, 1.5 * config.ball_diameter_px
        )
        self.tracker = TrackerV2(axis, config.target_position)
        self.frame_id = -1

    def _update_pipe(self, img, frame_id):
        state = self.pipe
        state.measured = False
        state.raw_count = 0
        state.score = 0.0
        if frame_id % POSE_INTERVAL_FRAMES == 0:
            minimum_pixels = max(
                8, int(0.10 * self.config.ball_diameter_px ** 2)
            )
            blobs = img.find_blobs(
                self.config.pipe_thresholds,
                roi=list(self.config.right_search_roi),
                x_stride=4,
                y_stride=3,
                area_threshold=minimum_pixels,
                pixels_threshold=minimum_pixels,
                merge=False,
                margin=0,
            )
            state.raw_count = len(blobs)
            if blobs:
                previous_y = state.axis.end.y
                blob = max(
                    blobs,
                    key=lambda item: (
                        float(item.pixels())
                        / (1.0 + abs(float(item.cyf()) - previous_y))
                    ),
                )
                measured_y = float(blob.cyf())
                if abs(measured_y - previous_y) <= (
                    3.0 * self.config.ball_diameter_px
                ):
                    filtered_y = previous_y + 0.55 * (
                        measured_y - previous_y
                    )
                    axis = Axis(
                        self.config.left_endpoint,
                        (self.config.right_endpoint[0], filtered_y),
                    )
                    self.tracker.set_axis(axis)
                    state.axis = axis
                    state.measured = True
                    state.acquired = True
                    state.age = 0
                    state.score = float(blob.pixels())

        if not state.measured:
            state.age += 1
        state.valid = (
            state.acquired and state.age <= POSE_COAST_FRAMES
        )
        state.roi = state.axis.roi(
            1.5 * self.config.ball_diameter_px,
            self.config.frame_width,
            self.config.frame_height,
        )

    def _local_roi(self, predicted):
        diameter = self.config.ball_diameter_px
        return clamp_roi(
            (
                predicted.x - 3.0 * diameter,
                predicted.y - 2.0 * diameter,
                6.0 * diameter,
                4.0 * diameter,
            ),
            self.config.frame_width,
            self.config.frame_height,
        )

    def _detect_ball(self, img, now_ms):
        predicted = self.tracker.predicted_point(now_ms)
        used_local = predicted is not None and self.tracker.misses == 0
        full_roi = self.pipe.roi
        search_roi = self._local_roi(predicted) if used_local else full_roi
        fell_back = predicted is not None and not used_local
        diameter = self.config.ball_diameter_px
        following = self.tracker.position_px is not None
        minimum_pixels = max(
            8,
            int(
                (0.06 if following else 0.12)
                * diameter
                * diameter
            ),
        )
        minimum_quality = (
            0.03 if following else 0.08
        ) * diameter * diameter
        blobs = img.find_blobs(
            self.config.ball_thresholds,
            roi=list(search_roi),
            x_stride=2 if used_local else 3,
            y_stride=2 if used_local else 3,
            area_threshold=minimum_pixels,
            pixels_threshold=minimum_pixels,
            merge=False,
            margin=0,
        )

        candidates = []
        minimum_side = 0.40 * diameter
        maximum_side = 1.80 * diameter
        maximum_lateral = max(8.0, 0.85 * diameter)
        for blob in blobs:
            width = float(blob.w())
            height = float(blob.h())
            pixels = float(blob.pixels())
            if (
                pixels < minimum_pixels
                or min(width, height) < minimum_side
                or max(width, height) > maximum_side
            ):
                continue
            density = pixels / max(1.0, width * height)
            aspect = max(width, height) / max(1.0, min(width, height))
            if density < 0.12 or aspect > 2.4:
                continue
            point = Point(blob.cxf(), blob.cyf())
            along, lateral = self.pipe.axis.project(point)
            if (
                along < 0.0
                or along > self.pipe.axis.length
                or abs(lateral) > maximum_lateral
            ):
                continue
            try:
                roundness = clamp(float(blob.roundness()), 0.0, 1.0)
            except Exception:
                roundness = 1.0 / aspect
            quality = (
                pixels
                * (0.5 + 0.5 * roundness)
                * (0.5 + 0.5 / aspect)
            )
            if quality < minimum_quality:
                continue
            candidates.append(
                Candidate(
                    point,
                    0.25 * (width + height),
                    quality,
                    along,
                    lateral,
                    blob,
                )
            )
        return DetectionResult(
            candidates,
            search_roi,
            full_roi,
            used_local,
            fell_back,
            len(blobs),
            self.pipe,
        )

    def process(self, img, now_ms, frame_id):
        self.frame_id = int(frame_id)
        self._update_pipe(img, self.frame_id)
        detection = self._detect_ball(img, now_ms)
        state = self.tracker.update(
            detection.candidates,
            now_ms,
            self.config.ball_diameter_px,
        )
        if not self.pipe.valid:
            state.valid = False
            state.measured = False
            state.coasting = False
        return legacy_detection(detection), legacy_state(state)


def legacy_detection(result):
    """Adapt typed V2 output to the existing preview/logging boundary."""
    pipe = result.pipe
    axis = pipe.axis
    pipe_dict = {
        "axis_start": axis.start.tuple(),
        "axis_end": axis.end.tuple(),
        "ball_roi": tuple(result.full_roi),
        "ball_quad": axis.quad(pipe.corridor),
        "measured": pipe.measured,
        "valid": pipe.valid,
        "age_frames": pipe.age,
        "raw_blob_count": pipe.raw_count,
        "score": pipe.score,
        "length": axis.length,
        "width": result.full_roi[3],
    }
    return {
        "candidates": [item.legacy_tuple() for item in result.candidates],
        "blobs": result.blobs,
        "circles": [],
        "circle_count": 0,
        "search_roi": tuple(result.search_roi),
        "full_roi": tuple(result.full_roi),
        "raw_count": result.raw_count,
        "fell_back": result.fell_back,
        "used_local": result.used_local,
        "axis_start": axis.start.tuple(),
        "axis_end": axis.end.tuple(),
        "roi_quad": pipe_dict["ball_quad"],
        "pipe": pipe_dict,
    }


def legacy_state(state):
    """Keep UART, telemetry and CSV formats stable during V2 rollout."""
    measurement = state.measurement
    return {
        "valid": state.valid,
        "measured": state.measured,
        "coasting": state.coasting,
        "x": state.point.x,
        "y": state.point.y,
        "radius": state.radius,
        "position": state.position,
        "position_px": state.position_px,
        "error_px": state.error_px,
        "lateral_px": int(round(state.lateral)),
        "velocity_px_s": state.velocity,
        "quality": state.quality,
        "measurement_x": (
            None if measurement is None else measurement.point.x
        ),
        "measurement_y": (
            None if measurement is None else measurement.point.y
        ),
        "measurement_radius": (
            None if measurement is None else measurement.radius
        ),
        "position_rejects": 0,
        "lateral_rejects": 0,
        "fixture_rejects": 0,
        "quality_rejects": 0,
        "jump_rejects": 0,
        "hits": state.hits,
        "misses": state.misses,
    }
