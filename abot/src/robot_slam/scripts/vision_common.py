#!/usr/bin/env python3
"""Pure-Python helpers shared by the visual recognition/tracking nodes."""

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, List, Optional


@dataclass(frozen=True)
class TemplateDetection:
    object_id: int
    width: float
    height: float
    center_x: float
    center_y: float
    area: float


def _map_point(matrix: Iterable[float], x: float, y: float) -> Optional[tuple]:
    h11, h12, h13, h21, h22, h23, h31, h32, h33 = matrix
    denominator = h13 * x + h23 * y + h33
    if not isfinite(denominator) or abs(denominator) < 1e-9:
        return None
    return (
        (h11 * x + h21 * y + h31) / denominator,
        (h12 * x + h22 * y + h32) / denominator,
    )


def _polygon_area(points: List[tuple]) -> float:
    area = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def parse_template_detections(objects: Iterable[float]) -> List[TemplateDetection]:
    """Parse find_object_2d's 12-float-per-object payload."""
    values = [float(value) for value in objects]
    detections: List[TemplateDetection] = []
    for start in range(0, len(values) - 11, 12):
        fields = values[start : start + 12]
        object_id = int(round(fields[0]))
        width = fields[1]
        height = fields[2]
        if width <= 0.0 or height <= 0.0:
            continue

        matrix = fields[3:12]
        corners = [
            _map_point(matrix, 0.0, 0.0),
            _map_point(matrix, width, 0.0),
            _map_point(matrix, width, height),
            _map_point(matrix, 0.0, height),
        ]
        if any(point is None for point in corners):
            continue

        finite_corners = [point for point in corners if point is not None]
        if not all(
            isfinite(coordinate)
            for point in finite_corners
            for coordinate in point
        ):
            continue

        center = _map_point(matrix, width * 0.5, height * 0.5)
        if center is None or not all(isfinite(value) for value in center):
            continue

        detections.append(
            TemplateDetection(
                object_id=object_id,
                width=width,
                height=height,
                center_x=center[0],
                center_y=center[1],
                area=_polygon_area(finite_corners),
            )
        )
    return detections


def select_template_detection(
    detections: Iterable[TemplateDetection], target_object_id: int = -1
) -> Optional[TemplateDetection]:
    candidates = list(detections)
    if target_object_id >= 0:
        candidates = [
            detection
            for detection in candidates
            if detection.object_id == target_object_id
        ]
    if not candidates:
        return None
    return max(candidates, key=lambda detection: (detection.area, -detection.object_id))


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalized_pixel_offset(center_x: float, image_width: int) -> float:
    if image_width <= 0:
        raise ValueError("image_width must be positive")
    return clamp(2.0 * center_x / float(image_width) - 1.0, -1.0, 1.0)


def normalized_bearing(bearing_radians: float, horizontal_fov_degrees: float) -> float:
    if horizontal_fov_degrees <= 0.0:
        raise ValueError("horizontal_fov_degrees must be positive")
    half_fov = horizontal_fov_degrees * 0.5
    return clamp(
        bearing_radians / (half_fov * 3.141592653589793 / 180.0), -1.0, 1.0
    )


def angular_velocity_for_error(
    normalized_error: float,
    deadband: float,
    angular_gain: float,
    max_angular_speed: float,
) -> float:
    error = clamp(normalized_error, -1.0, 1.0)
    deadband = clamp(abs(deadband), 0.0, 0.95)
    if abs(error) <= deadband:
        return 0.0

    sign = 1.0 if error > 0.0 else -1.0
    effective_error = sign * (abs(error) - deadband) / (1.0 - deadband)
    return clamp(
        -angular_gain * effective_error,
        -abs(max_angular_speed),
        abs(max_angular_speed),
    )


def smooth(previous: Optional[float], current: float, alpha: float) -> float:
    if previous is None:
        return current
    alpha = clamp(alpha, 0.0, 1.0)
    return previous + alpha * (current - previous)
