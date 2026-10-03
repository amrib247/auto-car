# HANDOFF — HBX 18859 RC Car Automation

**Updated:** 2026-10-02

## 1. Current project state

The active software project is the ROS 2 Humble workspace in [`auto-car-follow/`](auto-car-follow/). It contains two Python packages:

- `auto_car_sensors` connects to the ESP32-S3 camera's wireless MJPEG stream, runs YOLOv8, and publishes `vision_msgs/Detection2DArray` messages on `/vision/detections`.
- `auto_car_control` provides autonomous person-following and separate keyboard-driven WASD control. Both modes send normalized steering/throttle requests through a serial controller that converts them to voltage pairs.

The intended autonomous path is:

```text
Freenove ESP32-S3 CAM
  -> Wi-Fi SoftAP / HTTP MJPEG
  -> ROS 2 wireless_yolo_sensor / YOLOv8
  -> /vision/detections
  -> person_follow_controller / bounded PD steering
  -> USB serial voltage commands
  -> Feather + MCP4728 DAC
  -> stock transmitter analog controls
  -> stock RF link and HBX 18859
```

The vision, control, and serial-command software is present. **The end-to-end hardware path is not yet confirmed by the receiver firmware currently in this repository**; see [Hardware integration and safety](#4-hardware-integration-and-safety).

For ROS installation, WSL USB setup, dependencies, build/run commands, launch options, and detailed troubleshooting, use the project [`README.md`](auto-car-follow/README.md) as the primary setup guide. The standalone camera experiments and their guides remain in [`camera-test/`](camera-test/).

## 2. Camera and vision

The camera board is a Freenove ESP32-S3 CAM kit with an OV2640 sensor and OPI PSRAM. The wireless firmware in [`camera_wireless.ino`](camera-test/camera_wireless.ino) serves a QVGA MJPEG stream at `http://192.168.4.1:81/stream` when the host is connected to its SoftAP. The stream credentials are set in the firmware; consult the source rather than relying on stale copied values.

The ROS sensor node in [`sensor_node.py`](auto-car-follow/src/auto_car_sensors/auto_car_sensors/sensor_node.py):

- Loads the bundled `yolov8n.pt` model unless a model path is configured.
- Runs inference and publishes detections for all detected classes.
- Uses a background reader thread and a one-frame queue to avoid accumulating stale frames, and retries after stream failures.
- Displays an annotated preview by default; the preview can be disabled through the `display` parameter.
- Currently flips each frame vertically, without a horizontal mirror, before preview, inference, and publishing. This behavior is an in-progress working-tree change and should be checked against the physical camera orientation before relying on left/right steering.

ROS detections use source-image pixel coordinates. The controller assumes a 320-pixel-wide image by default. The sensor publishes empty detection messages while no frame is available, allowing the controller to stop rather than keep using an old target.

The standalone [`wireless_yolo_test.py`](camera-test/wireless_yolo_test.py) is an earlier host-side experiment. Its CUDA/FP16 setup is not the ROS sensor's documented runtime requirement: the ROS workspace README describes CPU PyTorch on WSL, with GPU acceleration optional. The wired serial camera sketch and receiver are separate experiments, not the active ROS vision path.

## 3. Control modes and operating behavior

### Autonomous person-follow

The autonomous controller in [`control_node.py`](auto-car-follow/src/auto_car_control/auto_car_control/control_node.py) selects the largest confident person detection, computes normalized horizontal error, and applies a bounded PD controller. Parameters—including steering sign and gains, confidence threshold, target timeout, serial port, and requested throttle—are in [`person_follow.yaml`](auto-car-follow/src/auto_car_control/config/person_follow.yaml).

Throttle is currently a fixed normalized request (default `0.1`) whenever a fresh person detection exists; otherwise it is neutral. This is **not distance control**: the car can continue at that throttle while any person remains detected. The initial gains and steering limit are starting values, not vehicle-tuned calibration.

The launch file is [`person_follow.launch.py`](auto-car-follow/src/auto_car_control/launch/person_follow.launch.py). There is a safety discrepancy to resolve: the current working-tree launch file defaults `enable_control` to `true`, while the README describes starting disarmed and explicitly enabling control. For camera/detection checks, pass `enable_control:=false` explicitly. Before vehicle testing, verify the launch default, serial device, receiver protocol, steering direction, and throttle calibration.

On an empty detection message or target timeout, the controller requests neutral. Graceful shutdown sends and flushes one final neutral command before closing serial. A failed or disconnected serial link cannot guarantee that a neutral command reaches the car.

### Manual WASD

The separate [`wasd_camera.launch.py`](auto-car-follow/src/auto_car_control/launch/wasd_camera.launch.py) starts the wireless camera sensor and keyboard controller. W/S control throttle, A/D steering, opposing keys cancel, and releasing keys returns the corresponding axis to neutral. ESC exits through the shutdown path.

Do not run manual and autonomous controller launches simultaneously: both write to the same serial controller. WSL keyboard capture uses `pynput` and requires an available WSLg/GUI session, as described in the workspace README.

## 4. Hardware integration and safety

The hardware notes below record the project's reported setup and measurements; recheck them against the physical build before wiring or powered testing.

- Vehicle: Haiboxing (HBX) 18859; the stock 2.4 GHz transmitter remains responsible for the RF link.
- Control board: Adafruit ESP32 Feather V2; DAC: MCP4728, 12-bit, with channel A used for steering and B for throttle in the documented wiring.
- Reported Feather I2C pins: SDA `A22`, SCL `A20`. The DAC and transmitter must share ground.
- Reported control ranges: steering center around 1.56 V; throttle neutral around 1.765 V, based on the current software calibration. The reported transmitter slider range is approximately 0–3.22 V.
- Camera power can be supplied over USB for bench testing or by a separate 5 V source for untethered tests. The camera is not yet documented as physically mounted for field operation.

The checked-in [`car_reciever.ino`](car-drive-test/car_reciever.ino) currently initializes the camera and serves its wireless HTTP stream; it is **not** the serial/DAC receiver described in the older handoff. Therefore, the receiver parser and the Feather-to-DAC-to-transmitter path must be located or implemented and verified before enabling control. Do not infer successful actuation from the Python serial sender alone.

The previous transmitter modification notes say the stock sliders were to be removed so the DAC directly drives their signal nodes, with the existing signal conditioning retained. Do not connect DAC outputs across intact sliders or to the transmitter's positive reference rail. Confirm the circuit and grounds before applying power.

Follow the safe progression in the workspace README: verify camera framing and detections; confirm the attached serial device and receiver protocol; test steering direction and neutral voltages with the drive wheels lifted; then conduct controlled vehicle tests in a clear area with a physical way to stop it. Treat software neutral commands as best-effort, not as an emergency stop or communications-loss failsafe.

## 5. Tests and project references

The workspace includes tests for controller logic, serial voltage mapping and shutdown, WASD key mapping, camera orientation, and the sensor reader. The README documents running them with:

```bash
colcon test --packages-select auto_car_control
colcon test-result --verbose
```

Also run the `auto_car_sensors` tests when changing camera or detection behavior. Test files are present, but this handoff does not assert that the current working tree has passed them.

Useful references:

- [`auto-car-follow/README.md`](auto-car-follow/README.md) — WSL setup, build/run workflow, launch instructions, parameters, and operating notes.
- [`camera-test/README.md`](camera-test/README.md), [`WIRELESSCAMERA.md`](camera-test/WIRELESSCAMERA.md), and [`WIREDCAMERA.md`](camera-test/WIREDCAMERA.md) — standalone camera reference workflows.
- [`mcp4278-test.ino`](MCP4278-test/mcp4278-test.ino) — MCP4728 output bench test.
- [`rc_controller.py`](car-drive-test/rc_controller.py) and [`calibration.py`](car-drive-test/calibration.py) — earlier serial-control and calibration references; check their values against the ROS package before reuse.

## 6. Recommended next steps

1. **Confirm the receiver path:** locate or write the firmware that parses newline-separated steering/throttle voltage pairs and updates the MCP4728; bench-test it without the truck moving.
2. **Reconcile safe startup:** make the person-follow launch default and README agree on whether control starts armed. Until then, explicitly launch with `enable_control:=false` for detection-only checks.
3. **Verify camera orientation and steering sign:** confirm the in-progress vertical-only frame flip against the mounted camera, then test left/right response with wheels lifted.
4. **Run the ROS package tests and an end-to-end bench check:** verify stream connection, `/vision/detections`, serial messages, and neutral behavior on no detection, timeout, and graceful shutdown.
5. **Tune vehicle behavior and mounting:** calibrate steering and throttle conservatively, decide how the car should regulate speed/distance to a person, and secure the camera and power source before untethered trials.
