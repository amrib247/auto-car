import queue
import threading
import unittest

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
    def test_reader_worker_keeps_ros_frame_queue_available_during_blocked_read(self):
        sensor = object.__new__(WirelessYoloSensor)
        sensor._reader_stop = threading.Event()
        sensor._reader_failed = threading.Event()
        sensor._frame_queue = queue.Queue(maxsize=1)
        capture = BlockingCapture()
        reader = threading.Thread(target=sensor._read_frames, args=(capture,))
        reader.start()

        try:
            self.assertTrue(capture.read_waiting.wait(timeout=1.0))
            self.assertIs(sensor._frame_queue.get(timeout=0.1), capture.frame)
        finally:
            sensor._reader_stop.set()
            capture.allow_read_to_finish.set()
            reader.join(timeout=1.0)

        self.assertFalse(reader.is_alive())


if __name__ == "__main__":
    unittest.main()