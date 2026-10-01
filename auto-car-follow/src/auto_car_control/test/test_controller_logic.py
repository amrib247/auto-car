import unittest

from auto_car_control.controller_logic import (
    DetectionBox,
    PDController,
    normalized_horizontal_error,
    select_largest_person,
)


class ControllerLogicTests(unittest.TestCase):
    def test_selects_largest_confident_person_only(self):
        detections = [
            DetectionBox("person", 30, 20, 10, 10, 0.9),
            DetectionBox("person", 80, 30, 20, 25, 0.8),
            DetectionBox("car", 50, 30, 100, 100, 0.99),
            DetectionBox("person", 50, 30, 100, 100, 0.2),
        ]
        self.assertEqual(select_largest_person(detections), detections[1])

    def test_returns_none_without_qualified_person(self):
        self.assertIsNone(select_largest_person([DetectionBox("car", 10, 10, 20, 20, 1.0)]))

    def test_normalizes_horizontal_error_and_applies_sign(self):
        self.assertEqual(normalized_horizontal_error(240, 320), 0.5)
        self.assertEqual(normalized_horizontal_error(240, 320, -1.0), -0.5)
        self.assertEqual(normalized_horizontal_error(500, 320), 1.0)

    def test_pd_uses_proportional_and_derivative_terms(self):
        controller = PDController(0.4, 0.02, output_limit=1.0)
        self.assertAlmostEqual(controller.compute(0.2, 1.0), 0.08)
        self.assertAlmostEqual(controller.compute(0.3, 1.1), 0.14)

    def test_pd_clamps_output_and_resets_derivative(self):
        controller = PDController(1.0, 0.5, output_limit=0.35)
        self.assertAlmostEqual(controller.compute(0.8, 1.0), 0.35)
        self.assertAlmostEqual(controller.compute(0.2, 2.0), 0.2)
        controller.reset()
        self.assertAlmostEqual(controller.compute(-0.1, 2.1), -0.1)


if __name__ == "__main__":
    unittest.main()