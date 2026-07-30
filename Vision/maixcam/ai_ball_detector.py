"""MaixHub YOLO steel-ball detector with a small temporal output filter."""

import math


def _clamp(value, low, high):
    return low if value < low else high if value > high else value


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
        self.start = Point(*start)
        self.end = Point(*end)
        dx = self.end.x - self.start.x
        dy = self.end.y - self.start.y
        self.length = math.hypot(dx, dy)
        if self.length <= 1.0:
            raise ValueError("AI coordinate axis is too short")
        self.ux = dx / self.length
        self.uy = dy / self.length

    def project(self, x, y):
        dx = float(x) - self.start.x
        dy = float(y) - self.start.y
        return (
            dx * self.ux + dy * self.uy,
            self.ux * dy - self.uy * dx,
        )

    def point(self, along, lateral):
        return Point(
            self.start.x + along * self.ux - lateral * self.uy,
            self.start.y + along * self.uy + lateral * self.ux,
        )


class AIVisionConfig:
    __slots__ = (
        "model_path",
        "frame_width",
        "frame_height",
        "axis_start",
        "axis_end",
        "target_position",
        "confidence",
        "valid_confidence",
        "iou",
        "coast_frames",
    )

    def __init__(
        self,
        model_path,
        frame_width,
        frame_height,
        axis_start,
        axis_end,
        target_position,
        confidence=0.25,
        valid_confidence=0.50,
        iou=0.45,
        coast_frames=2,
    ):
        self.model_path = str(model_path)
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.axis_start = tuple(axis_start)
        self.axis_end = tuple(axis_end)
        self.target_position = _clamp(float(target_position), 0.0, 1.0)
        self.confidence = _clamp(float(confidence), 0.01, 0.99)
        self.valid_confidence = _clamp(
            float(valid_confidence), self.confidence, 0.99
        )
        self.iou = _clamp(float(iou), 0.01, 0.99)
        self.coast_frames = max(0, int(coast_frames))


class StaticReference:
    __slots__ = ("axis", "roi")

    def __init__(self, axis, width, height):
        self.axis = axis
        self.roi = (0, 0, int(width), int(height))


class AIBox:
    __slots__ = ("x", "y", "w", "h", "score", "class_id", "label")

    def __init__(self, x, y, w, h, score, class_id, label):
        self.x = float(x)
        self.y = float(y)
        self.w = float(w)
        self.h = float(h)
        self.score = float(score)
        self.class_id = int(class_id)
        self.label = str(label)

    @property
    def center_x(self):
        return self.x + 0.5 * self.w

    @property
    def center_y(self):
        return self.y + 0.5 * self.h

    @property
    def radius(self):
        return 0.25 * (self.w + self.h)

    def packet(self):
        return (
            self.x,
            self.y,
            self.w,
            self.h,
            self.score,
            self.class_id,
        )


class AIBallDetector:
    """Run the MaixHub model and expose the existing control/monitor boundary."""

    __slots__ = (
        "config",
        "model",
        "labels",
        "pipe",
        "input_width",
        "input_height",
        "track_x",
        "track_y",
        "position_px",
        "lateral_px",
        "velocity_px_s",
        "radius",
        "last_ms",
        "hits",
        "misses",
        "quality",
    )

    def __init__(self, config, model=None):
        self.config = config
        if model is None:
            from maix import nn

            model = nn.YOLOv5(model=config.model_path)
        self.model = model
        self.labels = list(getattr(model, "labels", ()))
        self.input_width = int(model.input_width())
        self.input_height = int(model.input_height())
        self.pipe = StaticReference(
            Axis(config.axis_start, config.axis_end),
            config.frame_width,
            config.frame_height,
        )
        self.track_x = None
        self.track_y = None
        self.position_px = None
        self.lateral_px = 0.0
        self.velocity_px_s = 0.0
        self.radius = 0.0
        self.last_ms = None
        self.hits = 0
        self.misses = 0
        self.quality = 0.0

    def _boxes(self, img):
        network_image = img.resize(self.input_width, self.input_height)
        objects = self.model.detect(
            network_image,
            conf_th=self.config.confidence,
            iou_th=self.config.iou,
        )
        scale_x = float(self.config.frame_width) / self.input_width
        scale_y = float(self.config.frame_height) / self.input_height
        boxes = []
        for obj in objects:
            class_id = int(obj.class_id)
            label = (
                self.labels[class_id]
                if 0 <= class_id < len(self.labels)
                else str(class_id)
            )
            boxes.append(
                AIBox(
                    obj.x * scale_x,
                    obj.y * scale_y,
                    obj.w * scale_x,
                    obj.h * scale_y,
                    obj.score,
                    class_id,
                    label,
                )
            )
        return boxes

    def _empty_state(self):
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
            "quality": self.quality,
            "measurement_x": None,
            "measurement_y": None,
            "measurement_radius": None,
            "position_rejects": 0,
            "lateral_rejects": 0,
            "fixture_rejects": 0,
            "quality_rejects": 0,
            "jump_rejects": 0,
            "hits": self.hits,
            "misses": self.misses,
        }

    def _state(self, measured, measurement=None):
        axis = self.pipe.axis
        position = _clamp(self.position_px, 0.0, axis.length)
        point = axis.point(position, self.lateral_px)
        return {
            "valid": True,
            "measured": bool(measured),
            "coasting": not measured,
            "x": point.x,
            "y": point.y,
            "radius": self.radius,
            "position": position / axis.length,
            "position_px": position,
            "error_px": int(
                round(
                    position
                    - self.config.target_position * axis.length
                )
            ),
            "lateral_px": int(round(self.lateral_px)),
            "velocity_px_s": self.velocity_px_s,
            "quality": self.quality,
            "measurement_x": (
                None if measurement is None else measurement.center_x
            ),
            "measurement_y": (
                None if measurement is None else measurement.center_y
            ),
            "measurement_radius": (
                None if measurement is None else measurement.radius
            ),
            "position_rejects": 0,
            "lateral_rejects": 0,
            "fixture_rejects": 0,
            "quality_rejects": 0,
            "jump_rejects": 0,
            "hits": self.hits,
            "misses": self.misses,
        }

    def _update(self, boxes, now_ms):
        valid_boxes = [
            box
            for box in boxes
            if box.score >= self.config.valid_confidence
        ]
        if valid_boxes:
            selected = max(valid_boxes, key=lambda item: item.score)
            along, lateral = self.pipe.axis.project(
                selected.center_x, selected.center_y
            )
            along = _clamp(along, 0.0, self.pipe.axis.length)
            if self.position_px is not None and self.last_ms is not None:
                dt = max(
                    0.001,
                    min(0.2, (int(now_ms) - self.last_ms) / 1000.0),
                )
                measured_velocity = (along - self.position_px) / dt
                self.velocity_px_s += 0.35 * (
                    measured_velocity - self.velocity_px_s
                )
                self.position_px += 0.72 * (along - self.position_px)
                self.lateral_px += 0.72 * (
                    lateral - self.lateral_px
                )
                self.radius += 0.72 * (
                    selected.radius - self.radius
                )
            else:
                self.position_px = along
                self.lateral_px = lateral
                self.radius = selected.radius
                self.velocity_px_s = 0.0
            self.track_x = selected.center_x
            self.track_y = selected.center_y
            self.last_ms = int(now_ms)
            self.hits += 1
            self.misses = 0
            self.quality = 100.0 * selected.score
            return self._state(True, selected)

        self.hits = 0
        self.misses += 1
        if (
            self.position_px is None
            or self.misses > self.config.coast_frames
        ):
            return self._empty_state()
        if self.last_ms is not None:
            dt = max(
                0.0,
                min(0.2, (int(now_ms) - self.last_ms) / 1000.0),
            )
            self.position_px = _clamp(
                self.position_px + self.velocity_px_s * dt,
                0.0,
                self.pipe.axis.length,
            )
            self.last_ms = int(now_ms)
        self.quality *= 0.7
        return self._state(False)

    def process(self, img, now_ms, frame_id):
        boxes = self._boxes(img)
        state = self._update(boxes, now_ms)
        axis = self.pipe.axis
        candidates = [
            (
                box.center_x,
                box.center_y,
                box.radius,
                100.0 * box.score,
            )
            for box in boxes
        ]
        pipe_state = {
            "axis_start": axis.start.tuple(),
            "axis_end": axis.end.tuple(),
            "ball_roi": self.pipe.roi,
            "ball_quad": None,
            "measured": False,
            "valid": True,
            "age_frames": 0,
            "raw_blob_count": 0,
            "score": 0.0,
            "length": axis.length,
            "width": self.config.frame_height,
            "mode": "fixed_calibration",
        }
        detection = {
            "algorithm": "ai",
            "model": self.config.model_path,
            "candidates": candidates,
            "boxes": [box.packet() for box in boxes],
            "blobs": [],
            "circles": [],
            "circle_count": 0,
            "search_roi": self.pipe.roi,
            "full_roi": self.pipe.roi,
            "raw_count": len(boxes),
            "fell_back": False,
            "used_local": False,
            "axis_start": axis.start.tuple(),
            "axis_end": axis.end.tuple(),
            "roi_quad": None,
            "pipe": pipe_state,
        }
        return detection, state
