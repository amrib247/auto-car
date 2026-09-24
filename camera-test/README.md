# Camera Test Project

This folder contains the ESP32-S3 camera test setup for two different streaming approaches:

- Wired USB serial camera streaming
- Wireless Access Point (SoftAP) MJPEG streaming with optional YOLO object detection

Use the markdown guides in this folder as the primary reference before uploading firmware or running the Python scripts.

## Start here

### Wired setup
Read: [WIREDCAMERA.md](WIREDCAMERA.md)

This guide covers the direct USB serial workflow, including:
- ESP32-S3 camera capture
- JPEG frame packaging and serial transmission
- Python host-side decoding and display

Relevant files:
- [camera_wired.ino](camera_wired.ino) — Arduino firmware for the wired serial stream
- [camera_wired.py](camera_wired.py) — Python script that reads the serial stream and displays frames

### Wireless setup
Read: [WIRELESSCAMERA.md](WIRELESSCAMERA.md)

This guide covers the Wi-Fi camera workflow, including:
- ESP32-S3 as a Wi-Fi access point
- MJPEG HTTP streaming on port 81
- OpenCV stream verification
- YOLOv8 object detection and latency mitigation

Relevant files:
- [camera_wireless.ino](camera_wireless.ino) — Arduino firmware for the wireless MJPEG server
- [camera_wireless.py](camera_wireless.py) — basic Wi-Fi stream viewer
- [wireless_yolo_test.py](wireless_yolo_test.py) — real-time YOLO inference pipeline
- [yolov8n.pt](yolov8n.pt) — YOLOv8 Nano model weights used by the detection script

## File overview

- [camera_wired.ino](camera_wired.ino): ESP32-S3 firmware for the USB-connected camera stream.
- [camera_wired.py](camera_wired.py): Python receiver for the wired serial JPEG stream.
- [camera_wireless.ino](camera_wireless.ino): ESP32-S3 firmware for a wireless SoftAP MJPEG stream.
- [camera_wireless.py](camera_wireless.py): lightweight Python client for viewing the wireless stream.
- [wireless_yolo_test.py](wireless_yolo_test.py): advanced host-side script for YOLO object detection and frame refresh handling.
- [yolov8n.pt](yolov8n.pt): pre-trained YOLOv8 Nano model used for inference.
- [WIREDCAMERA.md](WIREDCAMERA.md): step-by-step instructions for the wired camera pipeline.
- [WIRELESSCAMERA.md](WIRELESSCAMERA.md): step-by-step instructions for the wireless camera and detection pipeline.

## Recommended workflow

1. Choose your camera architecture:
   - USB-connected serial stream → use the wired path
   - Wi-Fi access point stream → use the wireless path
2. Read the matching markdown file before programming or running code.
3. Upload the relevant Arduino sketch to the ESP32-S3.
4. Run the matching Python script on the host machine.

## Notes

- The wired setup is the simplest starting point for testing camera capture and frame decoding.
- The wireless setup is intended for untethered operation and adds computer vision processing.
- If you are testing detections or object tracking, use the wireless YOLO script and read [WIRELESSCAMERA.md](WIRELESSCAMERA.md) carefully.
