import unittest

import numpy as np

from auto_car_sensors.image_utils import orient_upside_down_camera


class ImageOrientationTests(unittest.TestCase):
    def test_rotates_frame_180_degrees(self):
        frame = np.array([[1, 2], [3, 4]], dtype=np.uint8)

        oriented = orient_upside_down_camera(frame)

        np.testing.assert_array_equal(oriented, np.array([[4, 3], [2, 1]], dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()