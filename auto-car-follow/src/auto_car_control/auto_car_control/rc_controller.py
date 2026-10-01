import serial
import time


class RCController:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)

        self.steer_min = 0.0
        self.steer_dz_lower = 1.56
        self.steer_rest = 1.56
        self.steer_dz_upper = 1.56
        self.steer_max = 3.1

        self.throttle_min = 0.0
        self.throttle_dz_lower = 1.37
        self.throttle_dz_upper = 2.16
        self.throttle_rest = (self.throttle_dz_lower + self.throttle_dz_upper) / 2
        self.throttle_max = 3.22

        self.current_steer_v = self.steer_rest
        self.current_throttle_v = self.throttle_rest
        self.last_throttle_intensity = 0.0
        self._closed = False

    def _set_steering_voltage(self, intensity):
        intensity = max(-1.0, min(1.0, intensity))
        if intensity == 0:
            self.current_steer_v = self.steer_rest
        elif intensity > 0:
            self.current_steer_v = self.steer_dz_upper + (
                intensity * (self.steer_max - self.steer_dz_upper)
            )
        else:
            self.current_steer_v = self.steer_dz_lower - (
                abs(intensity) * (self.steer_dz_lower - self.steer_min)
            )

    def _set_throttle_voltage(self, intensity):
        intensity = max(-1.0, min(1.0, intensity))
        if intensity == 0:
            self.current_throttle_v = self.throttle_rest
        elif intensity > 0:
            self.current_throttle_v = self.throttle_dz_upper + (
                intensity * (self.throttle_max - self.throttle_dz_upper)
            )
        else:
            self.current_throttle_v = self.throttle_dz_lower - (
                abs(intensity) * (self.throttle_dz_lower - self.throttle_min)
            )

    def set_steering(self, intensity):
        self._set_steering_voltage(intensity)
        self._send_command()

    def set_throttle(self, intensity):
        intensity = max(-1.0, min(1.0, intensity))
        if intensity < 0 and self.last_throttle_intensity >= 0:
            self.current_throttle_v = self.throttle_min
            self._send_command()
            time.sleep(0.05)

            self.current_throttle_v = self.throttle_rest
            self._send_command()
            time.sleep(0.05)

        self.last_throttle_intensity = intensity
        self._set_throttle_voltage(intensity)
        self._send_command()

    def set_controls(self, steering_intensity, throttle_intensity):
        steering_intensity = max(-1.0, min(1.0, steering_intensity))
        throttle_intensity = max(-1.0, min(1.0, throttle_intensity))
        self._set_steering_voltage(steering_intensity)

        if throttle_intensity < 0 and self.last_throttle_intensity >= 0:
            self.current_throttle_v = self.throttle_min
            self._send_command()
            time.sleep(0.05)
            self.current_throttle_v = self.throttle_rest
            self._send_command()
            time.sleep(0.05)

        self.last_throttle_intensity = throttle_intensity
        self._set_throttle_voltage(throttle_intensity)
        self._send_command()

    def _send_command(self):
        if self._closed:
            return
        command = f"{self.current_steer_v:.3f},{self.current_throttle_v:.3f}\n"
        self.ser.write(command.encode("utf-8"))

    def close(self):
        if self._closed:
            return
        try:
            self._set_steering_voltage(0.0)
            self._set_throttle_voltage(0.0)
            self.last_throttle_intensity = 0.0
            self._send_command()
            self.ser.flush()
        finally:
            try:
                self.ser.close()
            finally:
                self._closed = True