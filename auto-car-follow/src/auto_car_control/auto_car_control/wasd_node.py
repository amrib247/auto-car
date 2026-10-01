import threading

import rclpy
from pynput import keyboard
from rclpy.node import Node

from auto_car_control.rc_controller import RCController
from auto_car_control.wasd_logic import controls_from_pressed_keys


class WASDController(Node):
    def __init__(self) -> None:
        super().__init__("wasd_controller")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("control_rate_hz", 20.0)

        serial_port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        control_rate = float(self.get_parameter("control_rate_hz").value)
        if control_rate <= 0:
            raise ValueError("control_rate_hz must be positive")

        self._controller = RCController(serial_port, baudrate)
        self._pressed_keys: set[str] = set()
        self._key_lock = threading.Lock()
        self._closing = False
        self._serial_failed = False
        self._controller.set_controls(0.0, 0.0)
        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()
        self._timer = self.create_timer(1.0 / control_rate, self._control_tick)
        self.get_logger().info("WASD active: W/S throttle, A/D steering, ESC exits")

    @staticmethod
    def _key_name(key) -> str:
        try:
            return key.char.lower()
        except (AttributeError, TypeError):
            return ""

    def _on_press(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info("ESC pressed; returning controls to neutral")
            rclpy.try_shutdown()
            return False
        key_name = self._key_name(key)
        if key_name in {"w", "a", "s", "d"}:
            with self._key_lock:
                self._pressed_keys.add(key_name)

    def _on_release(self, key):
        key_name = self._key_name(key)
        if key_name in {"w", "a", "s", "d"}:
            with self._key_lock:
                self._pressed_keys.discard(key_name)

    def _control_tick(self) -> None:
        if self._closing or self._serial_failed:
            return
        if not self._listener.is_alive():
            self.get_logger().error("Keyboard listener stopped; returning controls to neutral")
            rclpy.try_shutdown()
            return

        with self._key_lock:
            pressed_keys = set(self._pressed_keys)
        steering, throttle = controls_from_pressed_keys(pressed_keys)
        try:
            self._controller.set_controls(steering, throttle)
        except Exception as error:
            self._serial_failed = True
            self.get_logger().fatal(f"Serial command failed; stopping manual control: {error}")
            rclpy.try_shutdown()

    def destroy_node(self):
        self._closing = True
        if hasattr(self, "_timer"):
            self._timer.cancel()
        if hasattr(self, "_listener"):
            self._listener.stop()
            self._listener.join(timeout=1.0)
        if hasattr(self, "_controller"):
            try:
                self._controller.close()
            except Exception as error:
                self.get_logger().error(f"Failed to send neutral command during shutdown: {error}")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WASDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()