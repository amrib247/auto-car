import queue
import threading
import unittest
from unittest.mock import Mock, patch

from auto_car_sensors.sensor_node import WirelessYoloSensor


class BlockingCapture:
    def __init__(self):
        self.frame = object()
        self._sent_frame = False
        self.read_waiting = threading.Event()
        self.allow_read_to_finish = threading.Event()

    def read(self):
        if not self._sent_frame:
            self._sent_frame = True
            return True, self.frame
        self.read_waiting.set()
        self.allow_read_to_finish.wait(timeout=2.0)
        return False, None


class SensorReaderTests(unittest.TestCase):
    def test_reader_worker_keeps_latest_frame_available_during_blocked_read(self):
        sensor = object.__new__(WirelessYoloSensor)
        sensor._reader_stop = threading.Event()
        sensor._reader_failed = threading.Event()
        sensor._frame_lock = threading.Lock()
        sensor._latest_frame = None
        sensor._latest_frame_sequence = 0
        sensor._latest_frame_received = 0.0
        capture = BlockingCapture()
        reader = threading.Thread(target=sensor._read_frames, args=(capture,))
        reader.start()

        try:
            self.assertTrue(capture.read_waiting.wait(timeout=1.0))
            with sensor._frame_lock:
                self.assertIs(sensor._latest_frame, capture.frame)
                self.assertEqual(sensor._latest_frame_sequence, 1)
                self.assertGreater(sensor._latest_frame_received, 0)
        finally:
            sensor._reader_stop.set()
            capture.allow_read_to_finish.set()
            reader.join(timeout=1.0)

        self.assertFalse(reader.is_alive())

    def test_disconnect_schedules_retry_using_short_configured_interval(self):
        sensor = object.__new__(WirelessYoloSensor)
        sensor._reader_stop = threading.Event()
        sensor._reader_thread = None
        capture = Mock()
        sensor._capture = capture
        sensor._reader_failed = threading.Event()
        sensor._frame_lock = threading.Lock()
        sensor._latest_frame = object()
        sensor._latest_frame_sequence = 2
        sensor._latest_frame_received = 1.0
        sensor._last_inference_sequence = 2
        sensor._preview_sequence = 2
        sensor._inference_lock = threading.Lock()
        sensor._latest_overlay = []
        sensor._inference_queue = queue.Queue(maxsize=1)
        sensor._reconnect_interval = 0.5

        with patch("auto_car_sensors.sensor_node.time.monotonic", return_value=10.0):
            sensor._disconnect()

        self.assertEqual(sensor._next_reconnect_time, 10.5)
        capture.release.assert_called_once()
        self.assertIsNone(sensor._latest_frame)


if __name__ == "__main__":
    unittest.main()