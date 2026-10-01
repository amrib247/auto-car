import os
import queue
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO

from auto_car_sensors.image_utils import orient_upside_down_camera


class WirelessYoloSensor(Node):
    def __init__(self) -> None:
        super().__init__("wireless_yolo_sensor")
        self.declare_parameter("stream_url", "http://192.168.4.1:81/stream")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("open_timeout_sec", 3.0)
        self.declare_parameter("read_timeout_sec", 1.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("display", True)

        stream_url = self.get_parameter("stream_url").value
        model_path = self.get_parameter("model_path").value
        confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._reconnect_interval = float(self.get_parameter("reconnect_interval_sec").value)
        self._open_timeout = float(self.get_parameter("open_timeout_sec").value)
        self._read_timeout = float(self.get_parameter("read_timeout_sec").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._display_enabled = bool(self.get_parameter("display").value)
        if min(publish_rate_hz, self._reconnect_interval, self._open_timeout, self._read_timeout) <= 0:
            raise ValueError("publish rate and camera timeouts must be positive")
        if not 0 <= confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be between 0 and 1")

        if not model_path:
            model_path = os.path.join(
                get_package_share_directory("auto_car_sensors"), "models", "yolov8n.pt"
            )
        self._model = YOLO(model_path)
        self._confidence_threshold = confidence_threshold
        self._stream_url = str(stream_url)
        self._capture = None
        self._reader_thread = None
        self._reader_stop = threading.Event()
        self._reader_failed = threading.Event()
        self._frame_queue = queue.Queue(maxsize=1)
        self._next_reconnect_time = 0.0
        self._connected_time = 0.0
        self._waiting_for_frame_logged = False
        self._preview_window = "RC Vision - YOLOv8"
        self._preview_started = False

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(Detection2DArray, "/vision/detections", qos)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._process_frame)
        self.get_logger().info(f"Loading camera stream from {self._stream_url}")

    def _publish_empty(self) -> None:
        message = Detection2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)

    def _connect(self) -> None:
        self._next_reconnect_time = time.monotonic() + self._reconnect_interval
        open_parameters = [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
            int(self._open_timeout * 1000),
            cv2.CAP_PROP_READ_TIMEOUT_MSEC,
            int(self._read_timeout * 1000),
        ]
        capture = cv2.VideoCapture(self._stream_url, cv2.CAP_FFMPEG, open_parameters)
        if not capture.isOpened():
            capture.release()
            self.get_logger().warning(f"Unable to open camera stream: {self._stream_url}")
            return
        self._capture = capture
        self._reader_stop.clear()
        self._reader_failed.clear()
        self._connected_time = time.monotonic()
        self._waiting_for_frame_logged = False
        self._reader_thread = threading.Thread(
            target=self._read_frames, args=(capture,), daemon=True
        )
        self._reader_thread.start()
        self.get_logger().info("Connected to wireless camera stream")

    def _read_frames(self, capture) -> None:
        while not self._reader_stop.is_set():
            received, frame = capture.read()
            if not received:
                self._reader_failed.set()
                return
            try:
                self._frame_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._frame_queue.put_nowait(frame)
                except queue.Full:
                    pass

    def _disconnect(self) -> None:
        self._reader_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=self._read_timeout + 0.5)
            self._reader_thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._reader_failed.clear()
        while True:
            try:
                self._frame_queue.get_nowait()
            except queue.Empty:
                break
        self._next_reconnect_time = time.monotonic() + self._reconnect_interval

    def _process_frame(self) -> None:
        if self._capture is None:
            if time.monotonic() >= self._next_reconnect_time:
                self._connect()
            if self._capture is None:
                self._publish_empty()
                return

        if self._reader_failed.is_set():
            self._disconnect()
            self.get_logger().warning("Camera frame unavailable; reconnecting")
            self._publish_empty()
            return
        try:
            frame = self._frame_queue.get_nowait()
        except queue.Empty:
            if (
                not self._waiting_for_frame_logged
                and time.monotonic() - self._connected_time >= self._read_timeout
            ):
                self._waiting_for_frame_logged = True
                self.get_logger().warning("Stream opened, but no camera frame has arrived yet")
            self._publish_empty()
            return
        frame = orient_upside_down_camera(frame)

        message = Detection2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        annotated_frame = frame.copy()
        if not self._show_preview(frame):
            return
        try:
            results = self._model(frame, conf=self._confidence_threshold, verbose=False)
            if results:
                annotated_frame = results[0].plot()
            for result in results:
                for box in result.boxes:
                    x_min, y_min, x_max, y_max = (float(value) for value in box.xyxy[0].tolist())
                    class_index = int(box.cls[0].item())
                    confidence = float(box.conf[0].item())
                    names = self._model.names
                    class_name = names.get(class_index, str(class_index)) if isinstance(names, dict) else names[class_index]

                    detection = Detection2D()
                    detection.header = message.header
                    detection.bbox.center.position.x = (x_min + x_max) / 2.0
                    detection.bbox.center.position.y = (y_min + y_max) / 2.0
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = max(0.0, x_max - x_min)
                    detection.bbox.size_y = max(0.0, y_max - y_min)
                    cv2.drawMarker(
                        annotated_frame,
                        (int(detection.bbox.center.position.x), int(detection.bbox.center.position.y)),
                        (0, 0, 255),
                        cv2.MARKER_CROSS,
                        20,
                        2,
                    )

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(class_name).lower()
                    hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis)
                    message.detections.append(detection)
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}")
        self._publisher.publish(message)
        self._show_preview(annotated_frame)

    def _show_preview(self, frame) -> bool:
        if not self._display_enabled:
            return True
        try:
            cv2.imshow(self._preview_window, frame)
            pressed_key = cv2.waitKey(1) & 0xFF
            if not self._preview_started:
                self._preview_started = True
                self.get_logger().info("Camera preview window opened")
            if pressed_key == ord("q"):
                self.get_logger().info("Preview closed; shutting down the sensor node")
                rclpy.try_shutdown()
                return False
        except cv2.error as error:
            self._display_enabled = False
            self.get_logger().warning(f"Camera preview unavailable; disabling display: {error}")
        return True

    def destroy_node(self):
        if hasattr(self, "_timer"):
            self._timer.cancel()
        self._disconnect()
        if self._display_enabled:
            try:
                cv2.destroyAllWindows()
            except cv2.error as error:
                self.get_logger().warning(f"Could not close camera preview cleanly: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = WirelessYoloSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()