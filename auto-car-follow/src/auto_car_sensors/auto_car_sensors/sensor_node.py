import os
import queue
import threading
import time

import cv2
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO

from auto_car_sensors.image_utils import orient_upside_down_camera


def validate_inference_device(device_name: str) -> str:
    try:
        device = torch.device(device_name)
    except (RuntimeError, ValueError) as error:
        raise ValueError(f"Invalid YOLO inference device {device_name!r}") from error

    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError(
                f"YOLO was configured to use {device}, but CUDA is unavailable "
                "in this Python environment. Install a CUDA-enabled PyTorch build "
                "and verify GPU access from WSL."
            )
        device_index = (
            torch.cuda.current_device() if device.index is None else device.index
        )
        if device_index >= torch.cuda.device_count():
            raise RuntimeError(
                f"YOLO device {device} requested, but only "
                f"{torch.cuda.device_count()} CUDA device(s) are available."
            )
    return str(device)


class WirelessYoloSensor(Node):
    def __init__(self) -> None:
        super().__init__("wireless_yolo_sensor")
        self.declare_parameter("stream_url", "http://192.168.0.203:81/stream")
        self.declare_parameter("model_path", "")
        self.declare_parameter("inference_device", "cuda:0")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("preview_rate_hz", 30.0)
        self.declare_parameter("reconnect_interval_sec", 0.5)
        self.declare_parameter("open_timeout_sec", 3.0)
        self.declare_parameter("read_timeout_sec", 5.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("display", True)

        stream_url = self.get_parameter("stream_url").value
        model_path = self.get_parameter("model_path").value
        self._inference_device = validate_inference_device(
            str(self.get_parameter("inference_device").value)
        )
        confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        preview_rate_hz = float(self.get_parameter("preview_rate_hz").value)
        self._reconnect_interval = float(self.get_parameter("reconnect_interval_sec").value)
        self._open_timeout = float(self.get_parameter("open_timeout_sec").value)
        self._read_timeout = float(self.get_parameter("read_timeout_sec").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._display_enabled = bool(self.get_parameter("display").value)
        if min(
            publish_rate_hz,
            preview_rate_hz,
            self._reconnect_interval,
            self._open_timeout,
            self._read_timeout,
        ) <= 0:
            raise ValueError("rates and camera timeouts must be positive")
        if not 0 <= confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be between 0 and 1")
        inference_device = torch.device(self._inference_device)
        if inference_device.type == "cuda":
            device_index = (
                torch.cuda.current_device()
                if inference_device.index is None
                else inference_device.index
            )
            device_name = torch.cuda.get_device_name(device_index)
            self.get_logger().info(
                f"YOLO CUDA inference enabled: {self._inference_device} ({device_name})"
            )
        else:
            self.get_logger().warning(
                f"YOLO inference is configured for {self._inference_device}, not CUDA"
            )

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
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_sequence = 0
        self._latest_frame_received = 0.0
        self._last_inference_sequence = 0
        self._latest_overlay = []
        self._inference_lock = threading.Lock()
        self._inference_queue = queue.Queue(maxsize=1)
        self._inference_stop = threading.Event()
        self._inference_thread = threading.Thread(
            target=self._inference_worker,
            name="yolo-inference",
            daemon=True,
        )
        self._next_reconnect_time = 0.0
        self._connected_time = 0.0
        self._waiting_for_frame_logged = False
        self._preview_window = "RC Vision - YOLOv8"
        self._preview_started = False
        self._preview_sequence = 0

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._publisher = self.create_publisher(Detection2DArray, "/vision/detections", qos)
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._process_frame)
        self._preview_timer = None
        if self._display_enabled:
            self._preview_timer = self.create_timer(
                1.0 / preview_rate_hz, self._show_latest_preview
            )
        self._inference_thread.start()
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
        with self._frame_lock:
            self._latest_frame = None
            self._latest_frame_sequence = 0
            self._latest_frame_received = 0.0
            self._last_inference_sequence = 0
            self._preview_sequence = 0
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
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_sequence += 1
                self._latest_frame_received = time.monotonic()

    def _disconnect(self) -> None:
        self._reader_stop.set()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=self._read_timeout + 0.5)
            self._reader_thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._reader_failed.clear()
        with self._frame_lock:
            self._latest_frame = None
            self._latest_frame_sequence = 0
            self._latest_frame_received = 0.0
            self._last_inference_sequence = 0
            self._preview_sequence = 0
        with self._inference_lock:
            self._latest_overlay = []
        while True:
            try:
                self._inference_queue.get_nowait()
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
        with self._frame_lock:
            frame = self._latest_frame
            sequence = self._latest_frame_sequence
            frame_received = self._latest_frame_received
            new_frame = sequence != self._last_inference_sequence and frame is not None
            if new_frame:
                self._last_inference_sequence = sequence

        if frame is None:
            if (
                not self._waiting_for_frame_logged
                and time.monotonic() - self._connected_time >= self._read_timeout
            ):
                self._waiting_for_frame_logged = True
                self.get_logger().warning("Stream opened, but no camera frame has arrived yet")
            self._publish_empty()
            return

        if time.monotonic() - frame_received > self._read_timeout:
            self._publish_empty()
            return

        if not new_frame:
            return

        inference_frame = orient_upside_down_camera(frame)
        try:
            self._inference_queue.put_nowait((inference_frame, sequence))
        except queue.Full:
            try:
                self._inference_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._inference_queue.put_nowait((inference_frame, sequence))
            except queue.Full:
                pass

    def _inference_worker(self) -> None:
        while not self._inference_stop.is_set():
            try:
                frame, _sequence = self._inference_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self._inference_stop.is_set():
                return
            self._infer_frame(frame)

    def _infer_frame(self, frame) -> None:
        message = Detection2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        preview_overlay = []
        try:
            results = self._model(
                frame,
                conf=self._confidence_threshold,
                device=self._inference_device,
                verbose=False,
            )
            for result in results:
                for box in result.boxes:
                    x_min, y_min, x_max, y_max = (
                        float(value) for value in box.xyxy[0].tolist()
                    )
                    class_index = int(box.cls[0].item())
                    confidence = float(box.conf[0].item())
                    names = self._model.names
                    class_name = (
                        names.get(class_index, str(class_index))
                        if isinstance(names, dict)
                        else names[class_index]
                    )

                    detection = Detection2D()
                    detection.header = message.header
                    detection.bbox.center.position.x = (x_min + x_max) / 2.0
                    detection.bbox.center.position.y = (y_min + y_max) / 2.0
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = max(0.0, x_max - x_min)
                    detection.bbox.size_y = max(0.0, y_max - y_min)

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(class_name).lower()
                    hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis)
                    message.detections.append(detection)
                    preview_overlay.append(
                        (x_min, y_min, x_max, y_max, str(class_name), confidence)
                    )
        except Exception as error:
            self.get_logger().error(f"YOLO inference failed: {error}")
        with self._inference_lock:
            self._latest_overlay = preview_overlay
        self._publisher.publish(message)

    def _show_latest_preview(self) -> None:
        if not self._display_enabled:
            return
        with self._frame_lock:
            frame = self._latest_frame
            sequence = self._latest_frame_sequence
        if frame is None:
            cv2.waitKey(1)
            return
        try:
            if sequence != self._preview_sequence:
                preview_frame = orient_upside_down_camera(frame).copy()
                with self._inference_lock:
                    overlay = list(self._latest_overlay)
                for x_min, y_min, x_max, y_max, class_name, confidence in overlay:
                    top_left = (int(x_min), int(y_min))
                    bottom_right = (int(x_max), int(y_max))
                    cv2.rectangle(preview_frame, top_left, bottom_right, (0, 255, 0), 2)
                    cv2.drawMarker(
                        preview_frame,
                        ((top_left[0] + bottom_right[0]) // 2,
                         (top_left[1] + bottom_right[1]) // 2),
                        (0, 0, 255),
                        cv2.MARKER_CROSS,
                        20,
                        2,
                    )
                    cv2.putText(
                        preview_frame,
                        f"{class_name} {confidence:.2f}",
                        (top_left[0], max(20, top_left[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )
                cv2.imshow(self._preview_window, preview_frame)
                self._preview_sequence = sequence
            pressed_key = cv2.waitKey(1) & 0xFF
            if not self._preview_started:
                self._preview_started = True
                self.get_logger().info("Camera preview window opened")
            if pressed_key == ord("q"):
                self.get_logger().info("Preview closed; shutting down the sensor node")
                rclpy.try_shutdown()
        except cv2.error as error:
            self._display_enabled = False
            self.get_logger().warning(f"Camera preview unavailable; disabling display: {error}")

    def destroy_node(self):
        if hasattr(self, "_timer"):
            self._timer.cancel()
        if self._preview_timer is not None:
            self._preview_timer.cancel()
        self._inference_stop.set()
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=5.0)
            if self._inference_thread.is_alive():
                self.get_logger().warning(
                    "YOLO inference worker did not stop before shutdown"
                )
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