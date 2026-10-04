import socket
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node

from auto_car_sensors.imu_estimator import ImuEstimator


class ImuUdpSensor(Node):
    def __init__(self) -> None:
        super().__init__("imu_udp_sensor")
        self.declare_parameter("udp_host", "0.0.0.0")
        self.declare_parameter("udp_port", 12345)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 1.5)
        self.declare_parameter("display", True)
        self.declare_parameter("frame_id", "imu_local")

        udp_host = str(self.get_parameter("udp_host").value)
        udp_port = int(self.get_parameter("udp_port").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._display_enabled = bool(self.get_parameter("display").value)
        if not 1 <= udp_port <= 65535:
            raise ValueError("udp_port must be between 1 and 65535")
        if publish_rate_hz <= 0 or self._stale_timeout <= 0:
            raise ValueError("publish rate and stale timeout must be positive")

        self._estimator = ImuEstimator()
        self._publisher = self.create_publisher(
            Vector3Stamped, "/imu/delta_position", 10
        )
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((udp_host, udp_port))
        except OSError:
            self._socket.close()
            raise
        self._socket.setblocking(False)

        self._invalid_packets = 0
        self._last_packet_warning = 0.0
        self._figure = None
        self._axis = None
        self._path_line = None
        self._position_marker = None
        self._status_text = None
        self._plt = None
        if self._display_enabled:
            self._start_plot()

        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._poll_udp_socket
        )
        self.get_logger().info(
            f"Listening for ICM-20948 UDP packets on {udp_host}:{udp_port}"
        )
        self.get_logger().info(
            "Publishing per-sample dx/dy increments on /imu/delta_position"
        )

    def _start_plot(self) -> None:
        try:
            import matplotlib.pyplot as plt

            plt.ion()
            self._plt = plt
            self._figure, self._axis = plt.subplots(figsize=(7, 7))
            (self._path_line,) = self._axis.plot(
                [], [], "-", linewidth=2, color="blue", label="Path"
            )
            self._position_marker = self._axis.scatter(
                [], [], color="red", zorder=5, label="Current position"
            )
            self._status_text = self._axis.text(
                0.02, 0.95, "", transform=self._axis.transAxes, fontsize=12, va="top"
            )
            self._axis.set_title("2D Spatial Trajectory with ZUPT")
            self._axis.set_xlabel("X Position (meters)")
            self._axis.set_ylabel("Y Position (meters)")
            self._axis.grid(True)
            self._axis.legend(loc="upper right")
            self._axis.set_aspect("equal", adjustable="datalim")
        except (ImportError, RuntimeError) as error:
            self._display_enabled = False
            self.get_logger().warning(
                f"IMU graph unavailable; continuing without display: {error}"
            )

    def _poll_udp_socket(self) -> None:
        while True:
            try:
                payload, address = self._socket.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError as error:
                self.get_logger().error(f"IMU UDP receive failed: {error}")
                return

            now = time.monotonic()
            was_calibrating = self._estimator.calibrating
            try:
                delta = self._estimator.process_datagram(payload, now)
            except (UnicodeDecodeError, ValueError) as error:
                self._invalid_packets += 1
                if now - self._last_packet_warning >= 5.0:
                    self.get_logger().warning(
                        f"Ignoring malformed IMU datagram from {address}: {error}"
                    )
                    self._last_packet_warning = now
                continue

            if was_calibrating and not self._estimator.calibrating:
                self.get_logger().info(
                    "IMU calibration complete; publishing displacement increments"
                )

            if delta is not None:
                message = Vector3Stamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = self._frame_id
                message.vector.x = delta[0]
                message.vector.y = delta[1]
                message.vector.z = 0.0
                self._publisher.publish(message)

        if self._display_enabled:
            self._update_plot()

    def _update_plot(self) -> None:
        if self._figure is None or self._plt is None:
            return
        if not self._plt.fignum_exists(self._figure.number):
            self.get_logger().info("IMU graph window closed; disabling graph")
            self._display_enabled = False
            return

        estimator = self._estimator
        self._path_line.set_data(estimator.px_data, estimator.py_data)
        self._position_marker.set_offsets([[estimator.px, estimator.py]])

        now = time.monotonic()
        if estimator.calibrating:
            status = (
                f"Calibrating: {estimator.calibration_count}/"
                f"{estimator.calibration_samples}\nKeep sensor still"
            )
            color = "darkorange"
        elif estimator.last_packet_time is None:
            status = "Waiting for IMU UDP data..."
            color = "darkorange"
        elif now - estimator.last_packet_time > self._stale_timeout:
            status = "IMU signal lost"
            color = "darkorange"
        elif estimator.is_stopped:
            status = "Status: STOPPED (ZUPT active)"
            color = "green"
        else:
            speed = (estimator.vx**2 + estimator.vy**2) ** 0.5
            status = f"Status: MOVING\nVelocity: {speed:.2f} m/s"
            color = "red"

        self._status_text.set_text(status)
        self._status_text.set_color(color)
        margin = 1.0
        self._axis.set_xlim(
            min(estimator.px_data) - margin, max(estimator.px_data) + margin
        )
        self._axis.set_ylim(
            min(estimator.py_data) - margin, max(estimator.py_data) + margin
        )
        self._figure.canvas.draw_idle()
        self._plt.pause(0.001)

    def destroy_node(self):
        if hasattr(self, "_timer"):
            self._timer.cancel()
        if hasattr(self, "_socket"):
            self._socket.close()
        if self._figure is not None and self._plt is not None:
            self._plt.close(self._figure)
        if self._invalid_packets:
            self.get_logger().warning(
                f"Ignored {self._invalid_packets} malformed IMU datagram(s)"
            )
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuUdpSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
