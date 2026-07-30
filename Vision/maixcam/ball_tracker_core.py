"""Hardware-independent 1-D prediction and candidate selection."""

import math


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def project_to_axis(x, y, axis_start, axis_end):
    """Return normalized position, signed lateral distance and projection."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    vx = float(x1 - x0)
    vy = float(y1 - y0)
    length_sq = vx * vx + vy * vy
    if length_sq <= 0.0:
        raise ValueError("axis start and end must be different")

    dx = float(x - x0)
    dy = float(y - y0)
    position = (dx * vx + dy * vy) / length_sq
    axis_length = math.sqrt(length_sq)
    lateral = (vx * dy - vy * dx) / axis_length
    projection_x = x0 + position * vx
    projection_y = y0 + position * vy
    return position, lateral, projection_x, projection_y, axis_length


def point_from_axis(position_px, lateral_px, axis_start, axis_end):
    """Convert along-axis and lateral pixel coordinates back to image x/y."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    vx = float(x1 - x0)
    vy = float(y1 - y0)
    axis_length = math.hypot(vx, vy)
    if axis_length <= 0.0:
        raise ValueError("axis start and end must be different")
    ux = vx / axis_length
    uy = vy / axis_length
    return (
        x0 + position_px * ux - lateral_px * uy,
        y0 + position_px * uy + lateral_px * ux,
    )


class BallTracker:
    """Track the ball along the pipe with an alpha-beta predictor.

    Candidates are ``(x, y, radius, quality)`` tuples.  A short measurement
    gap is coasted using velocity prediction, so a single weak/blurred frame
    does not immediately send ``none`` to the controller.
    """

    def __init__(
        self,
        axis_start,
        axis_end,
        target_position=0.5,
        max_axis_distance_px=25,
        max_below_axis_distance_px=None,
        max_frame_jump_px=80,
        acquire_position_margin=0.0,
        track_position_margin=0.02,
        acquire_endpoint_inset=0.0,
        track_endpoint_inset=0.0,
        acquire_min_quality=60.0,
        track_min_quality=0.0,
        smoothing_alpha=0.70,
        velocity_beta=0.12,
        lateral_alpha=0.55,
        confirm_frames=2,
        coast_frames=3,
        endpoint_coast_frames=None,
        memory_frames=8,
        fixture_exclusions=(),
        fixture_blob_override_quality=0.0,
        fixture_soft_radius_scale=0.0,
        fixture_soft_penalty_per_px=0.0,
        endpoint_snap_left_position=0.0,
        endpoint_snap_right_position=1.0,
        endpoint_snap_enter=0.0,
        endpoint_snap_exit=0.0,
        endpoint_snap_confirm_frames=1,
    ):
        self.axis_start = axis_start
        self.axis_end = axis_end
        self.target_position = target_position
        self.max_axis_distance_px = float(max_axis_distance_px)
        self.max_below_axis_distance_px = float(
            max_axis_distance_px
            if max_below_axis_distance_px is None
            else max_below_axis_distance_px
        )
        self.max_frame_jump_px = float(max_frame_jump_px)
        self.acquire_position_margin = clamp(
            float(acquire_position_margin), 0.0, 0.25
        )
        self.track_position_margin = clamp(
            float(track_position_margin), 0.0, 0.25
        )
        self.acquire_endpoint_inset = clamp(
            float(acquire_endpoint_inset), 0.0, 0.25
        )
        self.track_endpoint_inset = clamp(
            float(track_endpoint_inset), 0.0, 0.25
        )
        self.acquire_min_quality = max(0.0, float(acquire_min_quality))
        self.track_min_quality = max(0.0, float(track_min_quality))
        self.position_alpha = clamp(float(smoothing_alpha), 0.0, 1.0)
        self.velocity_beta = clamp(float(velocity_beta), 0.0, 1.0)
        self.lateral_alpha = clamp(float(lateral_alpha), 0.0, 1.0)
        self.confirm_frames = max(1, int(confirm_frames))
        self.coast_frames = max(0, int(coast_frames))
        self.endpoint_coast_frames = max(
            self.coast_frames,
            (
                self.coast_frames
                if endpoint_coast_frames is None
                else int(endpoint_coast_frames)
            ),
        )
        self.memory_frames = max(
            self.endpoint_coast_frames,
            int(memory_frames),
        )
        self.fixture_exclusions = tuple(
            (
                float(position),
                float(lateral),
                max(0.0, float(radius)),
            )
            for position, lateral, radius in fixture_exclusions
        )
        self.fixture_blob_override_quality = max(
            0.0, float(fixture_blob_override_quality)
        )
        self.fixture_soft_radius_scale = max(
            0.0, float(fixture_soft_radius_scale)
        )
        self.fixture_soft_penalty_per_px = max(
            0.0, float(fixture_soft_penalty_per_px)
        )
        self.endpoint_snap_left_position = clamp(
            float(endpoint_snap_left_position), 0.0, 0.49
        )
        self.endpoint_snap_right_position = clamp(
            float(endpoint_snap_right_position), 0.51, 1.0
        )
        self.endpoint_snap_enter = clamp(
            float(endpoint_snap_enter), 0.0, 0.49
        )
        self.endpoint_snap_exit = clamp(
            max(float(endpoint_snap_exit), self.endpoint_snap_enter),
            self.endpoint_snap_enter,
            0.49,
        )
        self.endpoint_snap_confirm_frames = max(
            1, int(endpoint_snap_confirm_frames)
        )
        self.axis_length = project_to_axis(
            axis_start[0], axis_start[1], axis_start, axis_end
        )[4]
        self.reset()

    def reset(self):
        self.position_px = None
        self.lateral_px = 0.0
        self.radius = None
        self.velocity_px_s = 0.0
        self.last_update_ms = None
        self.hits = 0
        self.misses = 0
        self.confirmed = False
        # Remains true for the short memory window after a confirmed track is
        # lost.  This lets a geometrically continuous, temporarily dim ball
        # use the tracking threshold without making first acquisition looser.
        self.trusted_memory = False
        self.last_quality = 0.0
        self.measurement_x = None
        self.measurement_y = None
        self.measurement_radius = None
        self.endpoint_lock = None
        self.endpoint_pending = None
        self.endpoint_pending_hits = 0
        self.selection_rejections = {
            "position": 0,
            "lateral": 0,
            "fixture": 0,
            "quality": 0,
            "jump": 0,
        }

    def has_track(self):
        return self.position_px is not None

    def set_axis(self, axis_start, axis_end):
        """Change pipe coordinates without teleporting the tracked image point.

        The filter state is stored as along-axis and lateral pixels.  When the
        pipe pose changes, reproject the same image-space point into the new
        coordinate system before processing the next measurement.
        """
        new_start = (float(axis_start[0]), float(axis_start[1]))
        new_end = (float(axis_end[0]), float(axis_end[1]))
        new_length = project_to_axis(
            new_start[0], new_start[1], new_start, new_end
        )[4]
        if new_length <= 0.0:
            raise ValueError("axis start and end must be different")

        old_length = self.axis_length
        image_point = None
        if self.position_px is not None:
            image_point = point_from_axis(
                self.position_px,
                self.lateral_px,
                self.axis_start,
                self.axis_end,
            )

        self.axis_start = new_start
        self.axis_end = new_end
        self.axis_length = new_length
        if image_point is None:
            return

        position, lateral, _, _, _ = project_to_axis(
            image_point[0], image_point[1], new_start, new_end
        )
        self.position_px = clamp(
            position * new_length,
            -self.track_position_margin * new_length,
            (1.0 + self.track_position_margin) * new_length,
        )
        self.lateral_px = float(lateral)
        if self.endpoint_lock == "left":
            self.position_px = (
                self.endpoint_snap_left_position * new_length
            )
            self.velocity_px_s = 0.0
        elif self.endpoint_lock == "right":
            self.position_px = (
                self.endpoint_snap_right_position * new_length
            )
            self.velocity_px_s = 0.0
        if old_length > 0.0:
            self.velocity_px_s *= new_length / old_length

    def _snap_endpoint_measurement(self, measurement_px):
        """Return a stable physical-stop measurement with hysteresis."""
        if self.endpoint_snap_enter <= 0.0:
            return measurement_px, False

        position = measurement_px / self.axis_length
        if self.endpoint_lock == "left":
            if position > self.endpoint_snap_exit:
                self.endpoint_lock = None
            else:
                return (
                    self.endpoint_snap_left_position * self.axis_length,
                    True,
                )
        elif self.endpoint_lock == "right":
            if position < 1.0 - self.endpoint_snap_exit:
                self.endpoint_lock = None
            else:
                return (
                    self.endpoint_snap_right_position * self.axis_length,
                    True,
                )

        side = None
        if position <= self.endpoint_snap_enter:
            side = "left"
        elif position >= 1.0 - self.endpoint_snap_enter:
            side = "right"

        if side is None:
            self.endpoint_pending = None
            self.endpoint_pending_hits = 0
            return measurement_px, False
        if self.endpoint_pending == side:
            self.endpoint_pending_hits += 1
        else:
            self.endpoint_pending = side
            self.endpoint_pending_hits = 1
        if self.endpoint_pending_hits < self.endpoint_snap_confirm_frames:
            return measurement_px, False

        self.endpoint_lock = side
        self.endpoint_pending = None
        self.endpoint_pending_hits = 0
        anchor = (
            self.endpoint_snap_left_position
            if side == "left"
            else self.endpoint_snap_right_position
        )
        return anchor * self.axis_length, True

    def _dt_seconds(self, now_ms):
        if self.last_update_ms is None or now_ms <= self.last_update_ms:
            return 0.0
        return min(0.20, (now_ms - self.last_update_ms) / 1000.0)

    def predicted_position_px(self, now_ms):
        if self.position_px is None:
            return None
        return clamp(
            self.position_px + self.velocity_px_s * self._dt_seconds(now_ms),
            -self.track_position_margin * self.axis_length,
            (1.0 + self.track_position_margin) * self.axis_length,
        )

    def predicted_point(self, now_ms):
        predicted = self.predicted_position_px(now_ms)
        if predicted is None:
            return None
        return point_from_axis(
            predicted, self.lateral_px, self.axis_start, self.axis_end
        )

    def _select_candidate(self, candidates, now_ms):
        accepted = []
        predicted = self.predicted_position_px(now_ms)
        acquiring = not self.confirmed
        self.selection_rejections = {
            "position": 0,
            "lateral": 0,
            "fixture": 0,
            "quality": 0,
            "jump": 0,
        }
        position_margin = (
            self.acquire_position_margin
            if acquiring
            else self.track_position_margin
        )
        endpoint_inset = (
            self.acquire_endpoint_inset
            if acquiring
            else self.track_endpoint_inset
        )
        # An endpoint inset is an explicit physical exclusion zone.  Keep the
        # older calibrated overhang margin behaviour when the inset is
        # disabled, but never let a non-zero inset be cancelled by that
        # margin.
        if endpoint_inset > 0.0:
            minimum_position = endpoint_inset
            maximum_position = 1.0 - endpoint_inset
        else:
            minimum_position = -position_margin
            maximum_position = 1.0 + position_margin
        min_quality = (
            self.track_min_quality
            if self.trusted_memory
            else self.acquire_min_quality
        )
        for candidate in candidates:
            x, y, radius, quality = candidate[:4]
            quality = float(quality)
            source = candidate[4] if len(candidate) > 4 else None
            fixture_x = candidate[5] if len(candidate) > 6 else x
            fixture_y = candidate[6] if len(candidate) > 6 else y
            position, lateral, _, _, _ = project_to_axis(
                x, y, self.axis_start, self.axis_end
            )
            if (
                position < minimum_position
                or position > maximum_position
            ):
                self.selection_rejections["position"] += 1
                continue
            if (
                lateral < -self.max_axis_distance_px
                or lateral > self.max_below_axis_distance_px
            ):
                self.selection_rejections["lateral"] += 1
                continue
            fixture_hit = False
            fixture_soft_cost = 0.0
            fixture_position_value, fixture_lateral_value, _, _, _ = (
                project_to_axis(
                    fixture_x,
                    fixture_y,
                    self.axis_start,
                    self.axis_end,
                )
            )
            for (
                fixture_position,
                fixture_lateral,
                fixture_radius,
            ) in self.fixture_exclusions:
                along_delta = (
                    fixture_position_value - fixture_position
                ) * self.axis_length
                lateral_delta = fixture_lateral_value - fixture_lateral
                distance = math.hypot(along_delta, lateral_delta)
                if distance <= fixture_radius:
                    fixture_hit = True
                    break
                soft_radius = (
                    fixture_radius * self.fixture_soft_radius_scale
                )
                if soft_radius > fixture_radius and distance < soft_radius:
                    fixture_soft_cost = max(
                        fixture_soft_cost,
                        (soft_radius - distance)
                        * self.fixture_soft_penalty_per_px,
                    )
            # A fixed-position mask is only a screw prior, not a blind area.
            # A strong LAB blob has direct appearance evidence for the larger
            # steel ball and is allowed to cross the same image region.
            fixture_override = (
                self.confirmed
                and
                source == 0
                and quality >= self.fixture_blob_override_quality
                and self.fixture_blob_override_quality > 0.0
            )
            if fixture_hit and not fixture_override:
                self.selection_rejections["fixture"] += 1
                continue
            if fixture_override:
                fixture_soft_cost = 0.0
            if quality < min_quality:
                self.selection_rejections["quality"] += 1
                continue

            position_px = position * self.axis_length
            radius_delta = (
                0.0 if self.radius is None else abs(float(radius) - self.radius)
            )

            if predicted is None or acquiring:
                # During acquisition, geometry and blob quality dominate.
                continuity = 0.0
            else:
                continuity = abs(position_px - predicted)
                if continuity > self.max_frame_jump_px:
                    self.selection_rejections["jump"] += 1
                    continue

            cost = (
                continuity
                + 1.6 * abs(lateral)
                + 1.8 * radius_delta
                + fixture_soft_cost
                - 0.04 * quality
            )
            accepted.append(
                (cost, candidate, position_px, float(lateral))
            )

        if not accepted:
            return None
        accepted.sort(key=lambda item: item[0])
        return accepted[0]

    def _state(self, valid, measured, quality=0.0):
        if self.position_px is None:
            return {
                "valid": False,
                "measured": False,
                "coasting": False,
                "x": 0.0,
                "y": 0.0,
                "radius": 0.0,
                "position": 0.0,
                "position_px": 0.0,
                "error_px": 0,
                "lateral_px": 0,
                "velocity_px_s": 0.0,
                "quality": 0.0,
                "measurement_x": None,
                "measurement_y": None,
                "measurement_radius": None,
                "position_rejects": self.selection_rejections["position"],
                "lateral_rejects": self.selection_rejections["lateral"],
                "fixture_rejects": self.selection_rejections["fixture"],
                "quality_rejects": self.selection_rejections["quality"],
                "jump_rejects": self.selection_rejections["jump"],
                "hits": self.hits,
                "misses": self.misses,
            }

        x, y = point_from_axis(
            self.position_px,
            self.lateral_px,
            self.axis_start,
            self.axis_end,
        )
        error_px = int(
            round(
                self.position_px
                - self.target_position * self.axis_length
            )
        )
        return {
            "valid": bool(valid),
            "measured": bool(measured),
            "coasting": bool(valid and not measured),
            "x": x,
            "y": y,
            "radius": self.radius,
            "position": self.position_px / self.axis_length,
            "position_px": self.position_px,
            "error_px": error_px,
            "lateral_px": int(round(self.lateral_px)),
            "velocity_px_s": self.velocity_px_s,
            "quality": float(quality),
            "measurement_x": self.measurement_x,
            "measurement_y": self.measurement_y,
            "measurement_radius": self.measurement_radius,
            "position_rejects": self.selection_rejections["position"],
            "lateral_rejects": self.selection_rejections["lateral"],
            "fixture_rejects": self.selection_rejections["fixture"],
            "quality_rejects": self.selection_rejections["quality"],
            "jump_rejects": self.selection_rejections["jump"],
            "hits": self.hits,
            "misses": self.misses,
        }

    def update(self, candidates, now_ms):
        """Update the filter and return the current tracking state."""
        selected = self._select_candidate(candidates, now_ms)
        if selected is None:
            self.measurement_x = None
            self.measurement_y = None
            self.measurement_radius = None
            self.hits = 0
            self.misses += 1
            if self.position_px is not None:
                predicted = self.predicted_position_px(now_ms)
                if predicted is not None:
                    self.position_px = predicted
                    self.last_update_ms = now_ms

            if self.confirmed:
                coast_limit = (
                    self.endpoint_coast_frames
                    if self.endpoint_lock is not None
                    else self.coast_frames
                )
                if self.misses <= coast_limit:
                    self.last_quality *= 0.65
                    return self._state(True, False, self.last_quality)
                # Once coasting is exhausted, a new measurement must pass
                # the acquisition gates and consecutive-frame confirmation.
                self.confirmed = False
                self.endpoint_lock = None
                self.endpoint_pending = None
                self.endpoint_pending_hits = 0

            if self.misses > self.memory_frames:
                self.reset()
            return self._state(False, False, 0.0)

        _, candidate, measurement_px, measurement_lateral = selected
        raw_x, raw_y, raw_radius, quality = candidate[:4]
        self.measurement_radius = float(raw_radius)
        measurement_px, endpoint_snapped = (
            self._snap_endpoint_measurement(measurement_px)
        )
        self.measurement_x, self.measurement_y = point_from_axis(
            measurement_px,
            measurement_lateral,
            self.axis_start,
            self.axis_end,
        )
        dt_s = self._dt_seconds(now_ms)

        if self.position_px is None:
            self.position_px = measurement_px
            self.lateral_px = measurement_lateral
            self.radius = float(raw_radius)
            self.velocity_px_s = 0.0
        elif endpoint_snapped:
            self.position_px = measurement_px
            self.velocity_px_s = 0.0
            self.lateral_px += self.lateral_alpha * (
                measurement_lateral - self.lateral_px
            )
            self.radius += self.position_alpha * (
                float(raw_radius) - self.radius
            )
        else:
            predicted = self.predicted_position_px(now_ms)
            residual = measurement_px - predicted
            self.position_px = predicted + self.position_alpha * residual
            if dt_s >= 0.001:
                self.velocity_px_s += (
                    self.velocity_beta * residual / dt_s
                )
            self.lateral_px += self.lateral_alpha * (
                measurement_lateral - self.lateral_px
            )
            self.radius += self.position_alpha * (
                float(raw_radius) - self.radius
            )

        self.position_px = clamp(
            self.position_px,
            -self.track_position_margin * self.axis_length,
            (1.0 + self.track_position_margin) * self.axis_length,
        )
        self.last_update_ms = now_ms
        self.misses = 0
        self.hits += 1
        if self.hits >= self.confirm_frames:
            self.confirmed = True
            self.trusted_memory = True
        self.last_quality = float(quality)
        return self._state(self.confirmed, True, quality)


def format_vision_line(state, output_scale=1.0):
    """Match the existing STM32 line protocol."""
    if not state["valid"]:
        return "none\n"
    scale = float(output_scale)
    error_px = int(round(float(state["error_px"]) * scale))
    lateral_px = int(round(float(state["lateral_px"]) * scale))
    return "{},{}\n".format(error_px, lateral_px)
