from dataclasses import dataclass
import math
from typing import Iterable, Optional


@dataclass(frozen=True)
class DetectionBox:
    class_id: str
    center_x: float
    center_y: float
    width: float
    height: float
    confidence: float


def select_largest_person(
    detections: Iterable[DetectionBox], min_confidence: float = 0.5
) -> Optional[DetectionBox]:
    people = []
    for detection in detections:
        if detection.class_id.strip().lower() not in {"person", "human", "0"}:
            continue
        values = (
            detection.center_x,
            detection.center_y,
            detection.width,
            detection.height,
            detection.confidence,
        )
        if not all(math.isfinite(value) for value in values):
            continue
        if detection.confidence < min_confidence or detection.width <= 0 or detection.height <= 0:
            continue
        people.append(detection)
    return max(people, key=lambda detection: detection.width * detection.height, default=None)


def normalized_horizontal_error(
    center_x: float, image_width: float, steering_sign: float = 1.0
) -> float:
    if not math.isfinite(center_x) or not math.isfinite(image_width) or image_width <= 0:
        raise ValueError("center_x must be finite and image_width must be positive")
    if steering_sign not in (-1.0, 1.0):
        raise ValueError("steering_sign must be -1.0 or 1.0")
    error = (2.0 * center_x / image_width) - 1.0
    return max(-1.0, min(1.0, error * steering_sign))


class PDController:
    def __init__(
        self,
        proportional_gain: float,
        derivative_gain: float,
        output_limit: float = 0.35,
        derivative_reset_interval: float = 0.5,
    ) -> None:
        values = (proportional_gain, derivative_gain, output_limit, derivative_reset_interval)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("PD parameters must be finite")
        if proportional_gain < 0 or derivative_gain < 0:
            raise ValueError("PD gains must be non-negative")
        if not 0 < output_limit <= 1:
            raise ValueError("output_limit must be in (0, 1]")
        if derivative_reset_interval <= 0:
            raise ValueError("derivative_reset_interval must be positive")

        self.proportional_gain = proportional_gain
        self.derivative_gain = derivative_gain
        self.output_limit = output_limit
        self.derivative_reset_interval = derivative_reset_interval
        self._previous_error: Optional[float] = None
        self._previous_time: Optional[float] = None

    def reset(self) -> None:
        self._previous_error = None
        self._previous_time = None

    def compute(self, error: float, timestamp: float) -> float:
        if not math.isfinite(error) or not math.isfinite(timestamp):
            raise ValueError("error and timestamp must be finite")

        derivative = 0.0
        if self._previous_time is not None and self._previous_error is not None:
            elapsed = timestamp - self._previous_time
            if 0 < elapsed <= self.derivative_reset_interval:
                derivative = (error - self._previous_error) / elapsed

        self._previous_error = error
        self._previous_time = timestamp
        output = self.proportional_gain * error + self.derivative_gain * derivative
        return max(-self.output_limit, min(self.output_limit, output))