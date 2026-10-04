import math
import statistics


class ImuEstimator:
    def __init__(
        self,
        calibration_samples: int = 200,
        window_size: int = 20,
        variance_threshold: float = 0.01,
    ) -> None:
        if calibration_samples <= 0 or window_size <= 0:
            raise ValueError("calibration_samples and window_size must be positive")
        if variance_threshold < 0:
            raise ValueError("variance_threshold must not be negative")

        self.calibration_samples = calibration_samples
        self.window_size = window_size
        self.variance_threshold = variance_threshold
        self.calibration_count = 0
        self.calibrating = True
        self.accel_bias = [0.0, 0.0, 0.0]
        self.gyro_bias = [0.0, 0.0, 0.0]

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.px = 0.0
        self.py = 0.0
        self.acceleration_window = []
        self.is_stopped = False
        self.last_sample_time = None
        self.last_packet_time = None
        self.px_data = [0.0]
        self.py_data = [0.0]

    def process_datagram(
        self, payload: bytes, received_at: float
    ) -> tuple[float, float] | None:
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

            if self.calibration_count == self.calibration_samples:
                self.accel_bias = [
                    bias / self.calibration_samples for bias in self.accel_bias
                ]
                self.gyro_bias = [
                    bias / self.calibration_samples for bias in self.gyro_bias
                ]
                self.calibrating = False
            return None

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
        if len(self.acceleration_window) > self.window_size:
            self.acceleration_window.pop(0)

        variance = (
            statistics.pvariance(self.acceleration_window)
            if len(self.acceleration_window) == self.window_size
            else 1.0
        )

        if variance < self.variance_threshold:
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

        previous_x = self.px
        previous_y = self.py
        self.px += self.vx * dt
        self.py += self.vy * dt
        dx = self.px - previous_x
        dy = self.py - previous_y

        self.px_data.append(self.px)
        self.py_data.append(self.py)
        if len(self.px_data) > 500:
            self.px_data.pop(0)
            self.py_data.pop(0)

        return dx, dy
