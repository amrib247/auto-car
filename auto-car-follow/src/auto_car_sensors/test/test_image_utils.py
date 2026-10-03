import unittest

import numpy as np

from auto_car_sensors.image_utils import orient_upside_down_camera


class ImageOrientationTests(unittest.TestCase):
    def test_flips_frame_vertically_without_mirroring_horizontally(self):
        frame = np.array([[1, 2], [3, 4]], dtype=np.uint8)

        oriented = orient_upside_down_camera(frame)

        np.testing.assert_array_equal(oriented, np.array([[3, 4], [1, 2]], dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()