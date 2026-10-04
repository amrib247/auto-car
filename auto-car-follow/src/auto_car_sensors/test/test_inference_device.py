import unittest
from unittest.mock import patch

from auto_car_sensors.sensor_node import validate_inference_device


class InferenceDeviceTests(unittest.TestCase):
    def test_cuda_device_is_rejected_if_pytorch_cannot_access_cuda(self):
        with patch("auto_car_sensors.sensor_node.torch.cuda.is_available", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "CUDA is unavailable"):
                validate_inference_device("cuda:0")

    def test_cuda_device_is_rejected_if_requested_index_is_missing(self):
        with (
            patch("auto_car_sensors.sensor_node.torch.cuda.is_available", return_value=True),
            patch("auto_car_sensors.sensor_node.torch.cuda.device_count", return_value=1),
        ):
            with self.assertRaisesRegex(RuntimeError, "only 1 CUDA device"):
                validate_inference_device("cuda:1")

    def test_cpu_device_is_preserved_when_explicitly_requested(self):
        self.assertEqual(validate_inference_device("cpu"), "cpu")

    def test_invalid_device_name_has_clear_error(self):
        with self.assertRaisesRegex(ValueError, "Invalid YOLO inference device"):
            validate_inference_device("not-a-device")


if __name__ == "__main__":
    unittest.main()
