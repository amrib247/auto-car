# ROS 2 Person-Following Car

Two ROS 2 Humble Python packages for WSL Ubuntu 22.04:

- `auto_car_sensors` reads the ESP32-CAM MJPEG stream, runs YOLOv8, publishes `vision_msgs/Detection2DArray` on `/vision/detections`, and displays an annotated OpenCV preview.
- `auto_car_control` follows the largest confident `person` detection with bounded PD steering and sends the paired steering/throttle voltages through the copied RC serial controller.

The camera publishes every detected class. Bounding boxes are in source-image pixels; the controller uses the configured 320-pixel image width. Steering error is positive for a target right of center by default. The steering sign, gains, limit, confidence threshold, target timeout, stream URL, model path, and serial port are configurable in `src/auto_car_control/config/person_follow.yaml`.

The camera is mounted upside down. The sensor flips every frame vertically, without mirroring it horizontally, before preview, inference, and publishing detections.

Throttle is the requested normalized command: `0.1` when a fresh person detection exists and `0` otherwise. With the current RC calibration, `0` maps to the neutral throttle voltage and `0.1` maps to about 2.266 V. This is not distance control: the car will continue at that throttle while any person remains detected. Use a clear test area and a physical way to stop the car.

## WSL setup

Install ROS 2 Humble for Ubuntu 22.04 in WSL and source its environment in each shell. Install ROS and system dependencies:

```bash
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-vision-msgs ros-humble-launch-ros \
  python3-colcon-common-extensions python3-rosdep python3-opencv python3-serial \
  python3-venv
```

The ROS console scripts use `/usr/bin/python3`, so install the inference dependencies into that interpreter's user site (a separate venv is not visible to the generated ROS executable). These commands select CPU-only PyTorch for WSL and keep setuptools in colcon's supported range:

```bash
python3 -m pip install --user "torch==2.14.1+cpu" "torchvision==0.29.1+cpu" \
  --extra-index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --user "setuptools==79.0.1"
python3 -m pip install --user "ultralytics==8.4.170" "opencv-python==4.11.0.86" pynput
python3 -c "import ultralytics, torch; print(ultralytics.__version__, torch.__version__)"
```

The model weights are bundled at `src/auto_car_sensors/models/yolov8n.pt`. PyTorch uses CPU by default when CUDA is unavailable; WSL GPU acceleration is optional.

If the workspace is stored on the Windows-mounted OneDrive drive, use its WSL path (typically `/mnt/c/Users/kille/OneDrive/Desktop/auto_car/auto-car-follow`). A Linux-native workspace path generally gives faster builds.

### Attach the USB controller in WSL

Windows does not automatically expose a USB-to-serial adapter to WSL 2. Install `usbipd-win` from an elevated Windows PowerShell:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

Reconnect the controller, then use elevated Windows PowerShell to find and bind its USB bus ID. The adapter in this setup is a CH9102 (`1a86:55d4`):

```powershell
usbipd list
usbipd bind --busid <BUSID>
```

Attach it to the running WSL distro from Windows PowerShell (this detaches it from Windows COM applications while attached):

```powershell
usbipd attach --wsl --busid <BUSID>
```

In WSL, verify that Linux sees the adapter and its serial device:

```bash
lsusb -d 1a86:55d4
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Use whichever device path exists as `serial_port:=...`. If opening it reports permission denied, add your WSL user to `dialout` with `sudo usermod -aG dialout "$USER"`, then restart the WSL distro. Reattach the USB device after restarting WSL.

## Build and run

From the workspace root in WSL:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Start with control disarmed; the control node sends neutral on startup and does not actuate until explicitly armed:

```bash
ros2 launch auto_car_control person_follow.launch.py \
  serial_port:=/dev/ttyUSB0 enable_control:=false
```

Inspect detections with `ros2 topic echo /vision/detections`. After verifying camera framing, steering direction, and the receiver protocol with the wheels lifted, arm the controller by starting the launch with `enable_control:=true`. Use `stream_url:=...`, `model_path:=...`, and `serial_port:=...` to override launch defaults.

### Manual WASD mode

Use the separate launch file for keyboard driving; it starts the camera sensor and WASD controller, but not the autonomous person-follow serial controller:

```bash
ros2 launch auto_car_control wasd_camera.launch.py serial_port:=/dev/ttyACM0
```

Hold W/S for forward/reverse and A/D for left/right; opposing keys cancel. Release keys to return that axis to neutral. Press ESC to send neutral and exit. The `pynput` listener uses WSLg, so keep the WSL GUI session available. Do not run the manual and autonomous control launches at the same time because both would write to the same serial controller.

The preview is enabled by default and can be disabled with `ros2 param set /wireless_yolo_sensor display false`. WSLg or another GUI display is required. The camera read runs in a background thread so a stalled stream does not block ROS callbacks; `Connected to wireless camera stream` only confirms the stream opened, while `Camera preview window opened` confirms a frame reached OpenCV. If no frame arrives before the configured read timeout, the node logs `Stream opened, but no camera frame has arrived yet`. Press `q` in the preview window to stop the sensor node; the controller's detection timeout returns steering and throttle to neutral.

## Serial and safety notes

The copied controller sends one newline-terminated `steering_voltage,throttle_voltage` pair per control tick at 115200 baud. The receiver source currently present in `car-drive-test/car_reciever.ino` is camera-server firmware rather than the described serial/DAC receiver, so confirm the actual receiver parser and voltage behavior before enabling control. The default PD gains and steering limit are conservative starting values, not vehicle-tuned calibration.

An empty detection message immediately returns outputs to neutral. If detections stop arriving, the 0.5-second target timeout also neutralizes outputs. A serial failure stops further command attempts; it cannot guarantee neutral if the physical link itself has failed.

On graceful control-node shutdown, the timer is canceled, one final neutral steering/throttle line is sent and flushed, then the serial port is closed. A forced process kill or failed serial link cannot guarantee command delivery.

## Tests

```bash
colcon test --packages-select auto_car_control
colcon test-result --verbose
```