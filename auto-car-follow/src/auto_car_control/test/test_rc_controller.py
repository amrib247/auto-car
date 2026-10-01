import unittest
from unittest.mock import Mock, patch

from auto_car_control.rc_controller import RCController


class RCControllerTests(unittest.TestCase):
    @patch("auto_car_control.rc_controller.serial.Serial")
    def test_combined_command_maps_normalized_inputs_to_voltage_line(self, serial_constructor):
        serial_port = Mock()
        serial_constructor.return_value = serial_port
        controller = RCController("COM-test")

        controller.set_controls(0.5, 0.1)

        serial_port.write.assert_called_once_with(b"2.330,2.266\n")

    @patch("auto_car_control.rc_controller.serial.Serial")
    def test_zero_inputs_map_to_steering_center_and_throttle_neutral(self, serial_constructor):
        serial_port = Mock()
        serial_constructor.return_value = serial_port
        controller = RCController("COM-test")

        controller.set_controls(0.0, 0.0)

        serial_port.write.assert_called_once_with(b"1.560,1.765\n")

    @patch("auto_car_control.rc_controller.serial.Serial")
    def test_close_sends_and_flushes_one_final_neutral_command(self, serial_constructor):
        serial_port = Mock()
        serial_constructor.return_value = serial_port
        controller = RCController("COM-test")
        controller.set_controls(0.5, 0.1)

        controller.close()
        controller.close()

        self.assertEqual(serial_port.write.call_args_list[-1].args[0], b"1.560,1.765\n")
        self.assertEqual(serial_port.write.call_count, 2)
        serial_port.flush.assert_called_once_with()
        serial_port.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()