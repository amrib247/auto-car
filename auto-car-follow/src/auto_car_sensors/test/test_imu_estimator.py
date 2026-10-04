import math
import unittest

from auto_car_sensors.imu_estimator import ImuEstimator


class ImuEstimatorTests(unittest.TestCase):
    def test_calibrates_from_stationary_samples_without_publishing_deltas(self):
        estimator = ImuEstimator(calibration_samples=3)
        stationary_sample = b"0.00,0.00,1000.00,0.00,0.00,0.00"

        for sample_index in range(3):
            delta = estimator.process_datagram(
                stationary_sample, sample_index * 0.02
            )
            self.assertIsNone(delta)

        self.assertFalse(estimator.calibrating)
        self.assertEqual(estimator.accel_bias, [0.0, 0.0, 0.0])
        self.assertEqual(estimator.gyro_bias, [0.0, 0.0, 0.0])

    def test_returns_per_sample_displacement_and_accumulates_plot_path(self):
        estimator = ImuEstimator(calibration_samples=1)
        estimator.process_datagram(
            b"0.00,0.00,1000.00,0.00,0.00,0.00", 0.0
        )

        delta = estimator.process_datagram(
            b"100.00,0.00,1000.00,0.00,0.00,0.00", 0.02
        )

        self.assertIsNotNone(delta)
        dx, dy = delta
        self.assertGreater(dx, 0.0)
        self.assertEqual(dy, 0.0)
        self.assertAlmostEqual(dx, estimator.px - estimator.px_data[-2])
        self.assertAlmostEqual(dy, estimator.py - estimator.py_data[-2])
        self.assertEqual(estimator.px_data[-1], estimator.px)
        self.assertEqual(estimator.py_data[-1], estimator.py)

    def test_rejects_malformed_and_non_finite_sensor_packets(self):
        estimator = ImuEstimator()

        with self.assertRaisesRegex(ValueError, "six comma-separated"):
            estimator.process_datagram(b"1,2,3", 0.0)
        with self.assertRaisesRegex(ValueError, "finite numbers"):
            estimator.process_datagram(b"nan,0,1000,0,0,0", 0.0)

    def test_clamps_elapsed_time_before_integrating(self):
        late_sample_estimator = ImuEstimator(calibration_samples=1)
        reference_estimator = ImuEstimator(calibration_samples=1)
        stationary_sample = b"0,0,1000,0,0,0"
        accelerated_sample = b"100,0,1000,0,0,0"

        for estimator in (late_sample_estimator, reference_estimator):
            estimator.process_datagram(stationary_sample, 0.0)
            estimator.process_datagram(accelerated_sample, 0.02)

        late_delta = late_sample_estimator.process_datagram(
            accelerated_sample, 10.0
        )
        reference_delta = reference_estimator.process_datagram(
            accelerated_sample, 0.12
        )

        self.assertTrue(math.isfinite(late_delta[0]))
        self.assertAlmostEqual(late_delta[0], reference_delta[0])
        self.assertAlmostEqual(late_sample_estimator.px, reference_estimator.px)

    def test_stationary_samples_trigger_zero_velocity_updates(self):
        estimator = ImuEstimator(calibration_samples=1, window_size=3)
        stationary_sample = b"0,0,1000,0,0,0"
        estimator.process_datagram(stationary_sample, 0.0)

        for sample_index in range(1, 4):
            delta = estimator.process_datagram(
                stationary_sample, sample_index * 0.02
            )

        self.assertTrue(estimator.is_stopped)
        self.assertEqual(delta, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
