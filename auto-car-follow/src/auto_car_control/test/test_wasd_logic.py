import unittest

from auto_car_control.wasd_logic import controls_from_pressed_keys


class WASDLogicTests(unittest.TestCase):
    def test_maps_wasd_to_normalized_controls(self):
        self.assertEqual(controls_from_pressed_keys({"w", "a"}), (-1.0, 1.0))
        self.assertEqual(controls_from_pressed_keys({"s", "d"}), (1.0, -1.0))

    def test_opposing_keys_cancel_and_released_keys_are_neutral(self):
        self.assertEqual(controls_from_pressed_keys({"w", "s", "a", "d"}), (0.0, 0.0))
        self.assertEqual(controls_from_pressed_keys(set()), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()