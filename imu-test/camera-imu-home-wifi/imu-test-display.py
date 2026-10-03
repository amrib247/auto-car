import math
import socket
import statistics
import threading
import time

import cv2
import matplotlib.pyplot as plt


CAMERA_URL = "http://192.168.0.203:81/stream"
UDP_HOST = "0.0.0.0"
UDP_PORT = 12345
MAX_POINTS = 500
MAX_CALIBRATION_SAMPLES = 200
WINDOW_SIZE = 20
VARIANCE_THRESHOLD = 0.01


class CameraReader:
    def __init__(self, stream_url: str) -> None:
        self.stream_url = stream_url
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.status = "Connecting to camera..."
        self.thread = threading.Thread(target=self._read_frames, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2)

    def get_frame_and_status(self):
        with self.lock:
            return self.latest_frame, self.status

    def _set_status(self, status: str) -> None:
        with self.lock:
            self.status = status

    def _read_frames(self) -> None:
        while not self.stop_event.is_set():
            capture = cv2.VideoCapture(self.stream_url)
            if not capture.isOpened():
                capture.release()
                self._set_status("Cannot open camera stream; retrying...")
                self.stop_event.wait(3)
                continue

            self._set_status("Camera connected")
            while not self.stop_event.is_set():
                received, frame = capture.read()
                if not received:
                    self._set_status("Camera stream interrupted; reconnecting...")
                    break
                with self.lock:
                    self.latest_frame = frame
                    self.status = "Camera connected"

            capture.release()
            if not self.stop_event.is_set():
                self.stop_event.wait(1)


class IMUTracker:
    def __init__(self) -> None:
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 0.0
        self.py = 0.0
        self.last_sample_time = None

        self.calibration_count = 0
        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrating = True
        self.acceleration_window = []
        self.is_stopped = False
        self.last_packet_time = None

        self.px_data = [0.0]
        self.py_data = [0.0]

    def process_datagram(self, payload: bytes, received_at: float) -> None:
        values = payload.decode("utf-8").strip().split(",")
        if len(values) != 6:
            raise ValueError("expected six comma-separated sensor values")

        sensor_values = [float(value) for value in values]
        if not all(math.isfinite(value) for value in sensor_values):
            raise ValueError("sensor values must be finite numbers")

        ax, ay, az = (value / 1000.0 for value in sensor_values[:3])
        gx, gy, gz = (math.radians(value) for value in sensor_values[3:])
        self.last_packet_time = received_at

        if self.calibrating:
            self.accel_bias[0] += ax
            self.accel_bias[1] += ay
            self.accel_bias[2] += az - 1.0
            self.gyro_bias[0] += gx
            self.gyro_bias[1] += gy
            self.gyro_bias[2] += gz
            self.calibration_count += 1

            if self.calibration_count == MAX_CALIBRATION_SAMPLES:
                self.accel_bias = [
                    bias / MAX_CALIBRATION_SAMPLES for bias in self.accel_bias
                ]
                self.gyro_bias = [
                    bias / MAX_CALIBRATION_SAMPLES for bias in self.gyro_bias
                ]
                self.calibrating = False
                print(
                    "Calibration complete.\n"
                    f"Accelerometer bias: {self.accel_bias}\n"
                    f"Gyroscope bias: {self.gyro_bias}"
                )
            return

        if self.last_sample_time is None:
            dt = 0.02
        else:
            dt = min(max(received_at - self.last_sample_time, 0.001), 0.1)
        self.last_sample_time = received_at

        ax -= self.accel_bias[0]
        ay -= self.accel_bias[1]
        az -= self.accel_bias[2]
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        accel_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
        alpha = 0.99 if abs(accel_magnitude - 1.0) > 0.2 else 0.96

        self.roll = alpha * (self.roll + gx * dt) + (1 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gy * dt) + (1 - alpha) * pitch_acc
        self.yaw += gz * dt

        linear_ax = (ax + math.sin(self.pitch)) * 9.81
        linear_ay = (ay - math.sin(self.roll) * math.cos(self.pitch)) * 9.81

        self.acceleration_window.append(
            math.sqrt(linear_ax**2 + linear_ay**2)
        )
        if len(self.acceleration_window) > WINDOW_SIZE:
            self.acceleration_window.pop(0)

        variance = (
            statistics.pvariance(self.acceleration_window)
            if len(self.acceleration_window) == WINDOW_SIZE
            else 1.0
        )

        if variance < VARIANCE_THRESHOLD:
            self.is_stopped = True
            self.vx = 0.0
            self.vy = 0.0
            linear_ax = 0.0
            linear_ay = 0.0
        else:
            self.is_stopped = False

        self.vx += linear_ax * dt
        self.vy += linear_ay * dt

        if abs(linear_ax) < 0.5:
            self.vx *= 0.75
        if abs(linear_ay) < 0.5:
            self.vy *= 0.75

        self.px += self.vx * dt
        self.py += self.vy * dt
        self.px_data.append(self.px)
        self.py_data.append(self.py)
        if len(self.px_data) > MAX_POINTS:
            self.px_data.pop(0)
            self.py_data.pop(0)


def create_imu_plot():
    figure, axis = plt.subplots(figsize=(7, 7))
    (path_line,) = axis.plot([], [], "-", linewidth=2, color="blue", label="Path")
    position_marker = axis.scatter([], [], color="red", zorder=5, label="Current position")
    status_text = axis.text(
        0.02, 0.95, "", transform=axis.transAxes, fontsize=12, va="top"
    )

    axis.set_title("2D Spatial Trajectory with ZUPT")
    axis.set_xlabel("X Position (meters)")
    axis.set_ylabel("Y Position (meters)")
    axis.grid(True)
    axis.legend(loc="upper right")
    axis.set_aspect("equal", adjustable="datalim")
    return figure, axis, path_line, position_marker, status_text


def update_imu_plot(tracker, axis, path_line, position_marker, status_text, now):
    path_line.set_data(tracker.px_data, tracker.py_data)
    position_marker.set_offsets([[tracker.px, tracker.py]])

    if tracker.calibrating:
        text = (
            f"Calibrating: {tracker.calibration_count}/"
            f"{MAX_CALIBRATION_SAMPLES}\nKeep sensor still"
        )
        color = "darkorange"
    elif tracker.last_packet_time is None:
        text = "Waiting for IMU UDP data..."
        color = "darkorange"
    elif now - tracker.last_packet_time > 1.5:
        text = "IMU signal lost"
        color = "darkorange"
    elif tracker.is_stopped:
        text = "Status: STOPPED (ZUPT active)"
        color = "green"
    else:
        speed = math.sqrt(tracker.vx**2 + tracker.vy**2)
        text = f"Status: MOVING\nVelocity: {speed:.2f} m/s"
        color = "red"

    status_text.set_text(text)
    status_text.set_color(color)

    margin = 1.0
    axis.set_xlim(min(tracker.px_data) - margin, max(tracker.px_data) + margin)
    axis.set_ylim(min(tracker.py_data) - margin, max(tracker.py_data) + margin)


def main() -> None:
    imu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = CameraReader(CAMERA_URL)
    figure, axis, path_line, position_marker, status_text = create_imu_plot()
    tracker = IMUTracker()
    invalid_packets = 0
    last_packet_warning = 0.0
    last_camera_status = None

    try:
        imu_socket.bind((UDP_HOST, UDP_PORT))
        imu_socket.setblocking(False)
        camera.start()
        print(f"Listening for IMU UDP packets on {UDP_HOST}:{UDP_PORT}")
        print(f"Opening camera stream: {CAMERA_URL}")
        print("Keep the IMU still while its initial 200-sample calibration runs.")
        print("Press q in the camera window or close the plot window to quit.")

        while plt.fignum_exists(figure.number):
            while True:
                try:
                    payload, _address = imu_socket.recvfrom(1024)
                except BlockingIOError:
                    break

                try:
                    tracker.process_datagram(payload, time.monotonic())
                except (UnicodeDecodeError, ValueError) as error:
                    invalid_packets += 1
                    now = time.monotonic()
                    if now - last_packet_warning >= 5:
                        print(f"Ignoring malformed IMU packet: {error}")
                        last_packet_warning = now

            now = time.monotonic()
            update_imu_plot(
                tracker, axis, path_line, position_marker, status_text, now
            )
            figure.canvas.draw_idle()

            frame, camera_status = camera.get_frame_and_status()
            if camera_status != last_camera_status:
                print(f"Camera: {camera_status}")
                last_camera_status = camera_status

            if frame is not None:
                display_frame = frame.copy()
                cv2.putText(
                    display_frame,
                    camera_status,
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("ESP32-S3 Camera", display_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            plt.pause(0.02)
    except OSError as error:
        raise SystemExit(f"Unable to start UDP listener: {error}") from error
    finally:
        camera.stop()
        imu_socket.close()
        cv2.destroyAllWindows()
        plt.close(figure)
        if invalid_packets:
            print(f"Ignored {invalid_packets} malformed IMU packet(s).")


if __name__ == "__main__":
    main()
