# HANDOFF — HBX 18859 RC TRANSMITTER CONTROL & VISION PIPELINE

Date: 2026-09-24

---

## 1. PROJECT OVERVIEW & ARCHITECTURE

The overall goal is to automate a Haiboxing (HBX) 18859 RC truck from a PC without modifying the truck's internal electronics. The system consists of two primary operational pipelines: a **Vision Subsystem** for object tracking and feedback, and a **Control Subsystem** for sending steering/throttle signals.

### Control Architecture

`PC Python` -> `USB Serial` -> `Adafruit ESP32 Feather V2` -> `I2C` -> `MCP4728 DAC` -> `Stock Transmitter Analog Inputs` -> `Stock 2.4 GHz RF` -> `Unmodified Truck`.

* The stock transmitter remains responsible for RF transmission.

### Vision Architecture (Active Wireless Stack)

`ESP32-S3 CAM (OV2640)` -> `Wi-Fi SoftAP (SSID: ESP32-CAM-RC)` -> `HTTP MJPEG Server (Port 81)` -> `PC Python (FreshFrameReader Thread)` -> `PyTorch CUDA (YOLOv8n FP16)` -> `Target Centroid Coordinates (x, y)`.

---

## 2. HARDWARE & SUBSYSTEM SUMMARY

### RC Vehicle & Transmitter Subsystem

* **Vehicle:** Haiboxing (HBX) 18859.
* **Stock Transmitter:** Powered by 2x AA batteries; PCB marking `LS-L103A-2.4GT, 2019-04-19, V5.00`.
* **Control Board:** Adafruit ESP32 Feather V2 (3.3V logic).
* **DAC:** MCP4728 4-channel, 12-bit I2C DAC.
* Pin mapping: `DAC A` -> Steering variable pad, `DAC B` -> Throttle variable pad.
* ESP32 I2C pins: `SDA = A22`, `SCL = A20`.



### Camera & Vision Subsystem

* **Board Model:** Freenove ESP32-S3 CAM Kit (FNK0085).
* **Sensor & Memory:** OV2640 camera module with 8MB onboard OPI PSRAM.
* **Network Mode:** Standalone Access Point (SoftAP) broadcasting SSID `ESP32-CAM-RC` (IP: `192.168.4.1`).
* **Powering Options:** USB connection to PC (desktop testing) or dedicated 5V USB power bank mounted on chassis (untethered operation).
* **Arduino IDE Tool Settings:**
* **Board:** `ESP32S3 Dev Module`
* **PSRAM:** `OPI PSRAM` *(Required for camera frame buffer allocation)*
* **USB CDC On Boot:** `Enabled`
* **Upload Mode:** `UART0 / Hardware CDC`



---

## 3. CONTROL SUBSYSTEM DETAILS (MCP4728 DAC)

### Measurements & Circuit Topology

* **Slider Signal Range:** Approximately 0 to 3.22 V; variable sides change linearly from 0 to 3.3 V.
* **Centers:** Steering center ~1.6 V, Throttle center ~1.7 V.
* **Reference Voltage:** Common positive slider reference is ~3.3 V relative to battery GND.
* **Signal Destinations:**
* Steering signal connects to bottom-row pin 4 of the unlabelled 16-pin RF IC.
* Throttle signal connects to bottom-row pin 5 of the unlabelled 16-pin RF IC.



### Circuit Modification Concept

```
BEFORE:
Common ~3.3 V rail ---> [Slider Source] ---> SIGNAL NODE ---> IC Input (Pins 4 / 5)
                                              |
                                       Existing R/C ---> GND

AFTER:
Former 3.3 V Slider Pad: OPEN (Disconnected)
MCP4728 OUT A/B ----------> SIGNAL NODE ---> IC Input (Pins 4 / 5)
                                              |
                                       Existing R/C ---> GND

```

> **CRITICAL:** Do NOT connect the DAC output across an intact slider or to the 3.3 V rail. The sliders must be physically removed so the DAC directly drives the variable signal pads.

### DAC Wiring & Grounding

* `ESP32 3V3` -> `MCP4728 VDD`
* `ESP32 GND` -> `MCP4728 GND` -> `Transmitter Battery Negative`
* `ESP32 A22 (SDA)` -> `MCP4728 SDA`
* `ESP32 A20 (SCL)` -> `MCP4728 SCL`
* `MCP4728 OUT A` -> Former Steering Signal Pad
* `MCP4728 OUT B` -> Former Throttle Signal Pad

---

## 4. VISION SUBSYSTEM DETAILS & SCRIPTS

### 4.1 Wireless Network & Stream Firmware (`camera_wireless.ino`)

The firmware configures the ESP32-S3 as a Wi-Fi Access Point and hosts an HTTP MJPEG server on port `81` at route `/stream`.

* **SSID:** `ESP32-CAM-RC` | **Password:** `password123`
* **Stream Endpoint:** `[http://192.168.4.1:81/stream](http://192.168.4.1:81/stream)`
* **Frame Settings:** `FRAMESIZE_QVGA` (320x240), `jpeg_quality = 12`, `fb_count = 2` in PSRAM.

### 4.2 Wireless Stream Verification (`wireless_camera.py`)

A basic verification script using standard OpenCV `VideoCapture("[http://192.168.4.1:81/stream](http://192.168.4.1:81/stream)")` to validate Wi-Fi connectivity and frame decoding without host serial cables.

### 4.3 AI Tracking & Low-Latency Performance Pipeline (`wireless_yolo_test.py`)

To prevent network video latency and enable real-time object tracking, the host pipeline incorporates two performance optimizations:

1. **PyTorch CUDA Acceleration:** Forces YOLOv8 Nano inference onto Nvidia GPU CUDA (`device='cuda'`) using half-precision floating point (`half=True`), reducing inference times significantly compared to CPU execution.
2. **Threaded Buffer Flushing (`FreshFrameReader`):** OpenCV's native `VideoCapture` buffers incoming network bytes in RAM. If frame processing takes longer than transmission rate, buffer accumulation creates a progressive 2–5 second video delay. The `FreshFrameReader` class runs a background thread that continuously grabs and discards stale frames, ensuring the main loop always processes the absolute newest live frame.
3. **Centroid Coordinate Extraction:** For every detected bounding box (`xyxy`), the script computes `center_x` and `center_y` to generate target tracking coordinates for steering and throttle calculations.

### 4.4 Legacy Wired Stack (Fallback Reference)

* **Firmware:** `camera_stream.ino` (Flushes custom binary frames `0xAA 0xBB` + 4-byte length + JPEG over USB Serial at 115200 baud).
* **Receiver:** `camera_viewer.py` (Parses binary serial buffers into OpenCV matrices).

---

## 5. Remote Car Testing

MCP4728 connected to Feather ESP32 V2, connected to remote control (as per about circuit diagram). Was able to control the car remotely via serial message from computer.

* **ESP32 Script:** `car_reciever.ino` Listens for serial inputs from RC controller class
* **RC Controller Class:** `rc_controller.py` Manages user input to car, sending commands via serial bus. Normalized and calibrated inputs and outputs


## 6. RECOMMENDED NEXT STEPS

1. **Closed-Loop Control Integration:**
* Map the YOLO target `center_x` coordinate relative to the frame center (160 px on QVGA) to calculate a steering error signal.
* Implement proportional or PID control mapping in Python to translate pixel error into MCP4728 DAC steering voltage output commands sent to the Adafruit Feather.


2. **Physical Untethered Mount:**
* Mount the ESP32-S3 CAM and a 5V USB power bank securely to the HBX 18859 chassis for mobile outdoor field testing.