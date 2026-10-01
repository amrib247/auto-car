import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2DArray

from auto_car_control.controller_logic import (
    DetectionBox,
    PDController,
    normalized_horizontal_error,
    select_largest_person,
)
from auto_car_control.rc_controller import RCController


class PersonFollowController(Node):
    def __init__(self) -> None:
        super().__init__("person_follow_controller")
        self.declare_parameter("detection_topic", "/vision/detections")
        self.declare_parameter("image_width", 320.0)
        self.declare_parameter("minimum_confidence", 0.5)
        self.declare_parameter("proportional_gain", 0.45)
        self.declare_parameter("derivative_gain", 0.015)
        self.declare_parameter("steering_limit", 0.35)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("throttle_when_person_detected", 0.1)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("enabled", False)

        image_width = float(self.get_parameter("image_width").value)
        minimum_confidence = float(self.get_parameter("minimum_confidence").value)
        steering_sign = float(self.get_parameter("steering_sign").value)
        target_timeout = float(self.get_parameter("target_timeout_sec").value)
        control_rate = float(self.get_parameter("control_rate_hz").value)
        throttle = float(self.get_parameter("throttle_when_person_detected").value)
        self._enabled = bool(self.get_parameter("enabled").value)
        if image_width <= 0 or target_timeout <= 0 or control_rate <= 0:
            raise ValueError("image_width, target_timeout_sec, and control_rate_hz must be positive")
        if not 0 <= minimum_confidence <= 1 or not 0 <= throttle <= 1:
            raise ValueError("minimum_confidence and throttle must be between 0 and 1")
        if steering_sign not in (-1.0, 1.0):
            raise ValueError("steering_sign must be -1.0 or 1.0")

        self._image_width = image_width
        self._minimum_confidence = minimum_confidence
        self._steering_sign = steering_sign
        self._target_timeout = target_timeout
        self._throttle_when_detected = throttle
        self._pd = PDController(
            float(self.get_parameter("proportional_gain").value),
            float(self.get_parameter("derivative_gain").value),
            float(self.get_parameter("steering_limit").value),
            derivative_reset_interval=target_timeout,
        )
        serial_port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        self._controller = RCController(serial_port, baudrate)
        self._target = None
        self._last_target_time = None
        self._last_steering = 0.0
        self._last_throttle = 0.0
        self._serial_failed = False
        self._closing = False

        self._controller.set_controls(0.0, 0.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = str(self.get_parameter("detection_topic").value)
        self._subscription = self.create_subscription(
            Detection2DArray, topic, self._on_detections, qos
        )
        self._timer = self.create_timer(1.0 / control_rate, self._control_tick)
        self.get_logger().info(f"Following person detections on {topic}")
        if not self._enabled:
            self.get_logger().warning("Control is disarmed; launch with enable_control:=true to arm")

    def _on_detections(self, message: Detection2DArray) -> None:
        boxes = []
        for detection in message.detections:
            if not detection.results:
                continue
            result = max(detection.results, key=lambda item: item.hypothesis.score)
            boxes.append(
                DetectionBox(
                    class_id=result.hypothesis.class_id,
                    center_x=float(detection.bbox.center.position.x),
                    center_y=float(detection.bbox.center.position.y),
                    width=float(detection.bbox.size_x),
                    height=float(detection.bbox.size_y),
                    confidence=float(result.hypothesis.score),
                )
            )

        self._target = select_largest_person(boxes, self._minimum_confidence)
        if self._target is None:
            self._last_target_time = None
            self._pd.reset()
            self._stop_if_needed()
        else:
            self._last_target_time = time.monotonic()

    def _control_tick(self) -> None:
        if self._serial_failed or self._closing:
            return
        if not self._enabled:
            self._stop_if_needed()
            return
        if self._target is None or self._last_target_time is None:
            self._stop_if_needed()
            return

        now = time.monotonic()
        if now - self._last_target_time > self._target_timeout:
            self._target = None
            self._last_target_time = None
            self._pd.reset()
            self.get_logger().warning("Person detection timed out; returning controls to neutral")
            self._stop_if_needed()
            return

        error = normalized_horizontal_error(
            self._target.center_x, self._image_width, self._steering_sign
        )
        steering = self._pd.compute(error, now)
        self._send_controls(steering, self._throttle_when_detected)

    def _stop_if_needed(self) -> None:
        if self._last_steering != 0.0 or self._last_throttle != 0.0:
            self._send_controls(0.0, 0.0)

    def _send_controls(self, steering: float, throttle: float) -> None:
        if self._serial_failed:
            return
        try:
            self._controller.set_controls(steering, throttle)
        except Exception as error:
            self._serial_failed = True
            self._last_steering = 0.0
            self._last_throttle = 0.0
            self.get_logger().fatal(f"Serial command failed; autonomous output stopped: {error}")
            return
        self._last_steering = steering
        self._last_throttle = throttle

    def destroy_node(self):
        self._closing = True
        if hasattr(self, "_timer"):
            self._timer.cancel()
        try:
            self._controller.close()
        except Exception as error:
            self.get_logger().error(f"Failed to send neutral command during shutdown: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()