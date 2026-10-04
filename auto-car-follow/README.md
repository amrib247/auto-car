# ROS 2 Person-Following Car

Two ROS 2 Humble Python packages for WSL Ubuntu 22.04:

- `auto_car_sensors` reads the ESP32-CAM MJPEG stream, runs YOLOv8, publishes `vision_msgs/Detection2DArray` on `/vision/detections`, and displays an annotated OpenCV preview. Its IMU node receives ICM-20948 UDP samples, publishes stamped per-sample displacement on `/imu/delta_position`, and plots the accumulated 2D path.
- `auto_car_control` follows the largest confident `person` detection with bounded PD steering and sends the paired steering/throttle voltages through the copied RC serial controller.

The camera publishes every detected class. Bounding boxes are in source-image pixels; the controller uses the configured 320-pixel image width. Steering error is positive for a target right of center by default. The steering sign, gains, limit, confidence threshold, target timeout, stream URL, model path, and serial port are configurable in `src/auto_car_control/config/person_follow.yaml`.

The camera is mounted upside down. The sensor flips every frame vertically, without mirroring it horizontally, before preview, inference, and publishing detections.

Throttle is the requested normalized command: `0.1` when a fresh person detection exists and `0` otherwise. With the current RC calibration, `0` maps to the neutral throttle voltage and `0.1` maps to about 2.266 V. This is not distance control: the car will continue at that throttle while any person remains detected. Use a clear test area and a physical way to stop the car.

## WSL setup

Install ROS 2 Humble for Ubuntu 22.04 in WSL and source its environment in each shell. Install ROS and system dependencies:

```bash
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-vision-msgs ros-humble-geometry-msgs \
  ros-humble-launch-ros python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-matplotlib python3-serial python3-venv
```

The ROS console scripts use `/usr/bin/python3`, so install inference dependencies into that interpreter's user site (a separate venv is not visible to the generated ROS executable). For the WSL GPU setup, install the CUDA-enabled PyTorch wheels rather than the CPU-only build. The CUDA 12.8 wheels work with a WSL NVIDIA driver that reports CUDA 13.x:

```bash
python3 -m pip install --user --upgrade --force-reinstall torch torchvision \
  --index-url https://download.pytorch.org/whl/cu128
python3 -m pip install --user "setuptools==79.0.1"
python3 -m pip install --user "ultralytics==8.4.170" "opencv-python==4.11.0.86" pynput
python3 -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CUDA unavailable')"
```

The model weights are bundled at `src/auto_car_sensors/models/yolov8n.pt`. The YOLO node defaults to `inference_device:=cuda:0`, passes that device explicitly to Ultralytics, logs the selected GPU, and fails at startup if CUDA is unavailable instead of silently falling back to CPU. Confirm the PyTorch check above reports `True` before launching. Use `inference_device:=cpu` only when intentionally running without CUDA.

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

The ESP32-S3 joins the home Wi-Fi network. Its current stream address is `http://192.168.0.203:81/stream`; reserve that address for the ESP32 in the router or override it with `stream_url:=...` if the address changes. The ESP32 sends six comma-separated accelerometer/gyroscope values as UDP datagrams to the laptop's home-network IPv4 address on port `12345`. The ROS IMU node receives these packets directly inside WSL; no Windows UDP relay is needed when WSL mirrored networking and its inbound firewall rule are configured as below.

### Allow ESP32 UDP packets into WSL

WSL 2's default NAT mode may not forward unsolicited UDP datagrams from LAN devices to Linux. On Windows 11 22H2 or newer, mirrored networking allows WSL to receive LAN traffic directly.

1. In PowerShell, check that WSL is current:

   ```powershell
   wsl --version
   wsl --update
   ```

2. Open `%UserProfile%\.wslconfig`. Preserve existing settings and add `networkingMode=mirrored` under the existing `[wsl2]` section (create that section if needed):

   ```ini
   [wsl2]
   networkingMode=mirrored
   ```

3. In **Administrator PowerShell**, allow inbound UDP only on the IMU port:

   ```powershell
   New-NetFirewallHyperVRule `
     -Name "WSL-IMU-UDP-12345" `
     -DisplayName "Allow IMU UDP to WSL" `
     -Direction Inbound `
     -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' `
     -Protocol UDP `
     -LocalPorts 12345
   ```

4. Restart WSL so the networking change takes effect:

   ```powershell
   wsl --shutdown
   ```

   Reopen Ubuntu, rebuild/source the workspace if needed, and launch the ROS sensor. Confirm that the ESP32 sketch's `LAPTOP_IP` is the laptop's current home-network IPv4 address and that the Windows firewall allows its LAN traffic. Stop any Windows IMU test listener or UDP relay before testing; these also bind port `12345`.

5. Keep the IMU still for the initial 200-sample calibration. The ROS launch should log that it is listening on `0.0.0.0:12345` and that calibration completed. Confirm that samples are being published:

   ```bash
   ros2 topic hz /imu/delta_position
   ros2 topic echo /imu/delta_position
   ```

   If the topic exists but has no messages, stop the ROS launch temporarily and test port `12345` with a small UDP listener inside WSL. If that listener receives nothing while the Windows IMU test receives packets, check mirrored networking, the Hyper-V firewall rule, and the ESP32's destination IP. A Windows-to-WSL UDP relay remains a fallback for systems where mirrored mode is unavailable.

Both camera launch files start the YOLO camera sensor and the IMU UDP sensor. The camera sensor keeps vertically flipping frames, running YOLO, publishing `/vision/detections`, and showing its annotated camera preview. The IMU sensor publishes each estimated displacement increment as `geometry_msgs/msg/Vector3Stamped` on `/imu/delta_position`: `vector.x` is `dx` and `vector.y` is `dy`, in meters for each received sample. It also accumulates those increments internally for the 2D trajectory graph. The graph's calibration and stopped/moving status follow the IMU test estimator. Keep the sensor still while its initial 200 samples are used for calibration.

Start with control disarmed; the control node sends neutral on startup and does not actuate until explicitly armed:

```bash
ros2 launch auto_car_control person_follow.launch.py \
  serial_port:=/dev/ttyUSB0 enable_control:=false \
  stream_url:=http://192.168.0.203:81/stream imu_udp_port:=12345
```

Inspect detections and IMU increments with:

```bash
ros2 topic echo /vision/detections
ros2 topic echo /imu/delta_position
```

The IMU plot is enabled by default and can be disabled for headless operation with `imu_display:=false`. The camera preview remains controlled by its existing `display` ROS parameter. Use `inference_device:=...` to override the YOLO device. After verifying camera framing, IMU axes, steering direction, and the receiver protocol with the wheels lifted, arm the controller by starting the launch with `enable_control:=true`. Use `stream_url:=...`, `model_path:=...`, `inference_device:=...`, `serial_port:=...`, `imu_udp_host:=...`, and `imu_udp_port:=...` to override launch defaults.

### Manual WASD mode

Use the separate launch file for keyboard driving; it starts the camera sensor, IMU UDP sensor, and WASD controller, but not the autonomous person-follow serial controller:

```bash
ros2 launch auto_car_control wasd_camera.launch.py serial_port:=/dev/ttyACM0
```

Hold W/S for forward/reverse and A/D for left/right; opposing keys cancel. Release keys to return that axis to neutral. Press ESC to send neutral and exit. The `pynput` listener uses WSLg, so keep the WSL GUI session available. Do not run the manual and autonomous control launches at the same time because both would write to the same serial controller.

The camera preview and YOLO inference run independently: the preview displays the newest camera frame at up to `preview_rate_hz` (default 30 Hz), while YOLO analyzes frames at `publish_rate_hz` (default 10 Hz) on a worker thread. The preview overlays the latest YOLO boxes. Both rates can be adjusted in `person_follow.yaml`. The preview is enabled by default and can be disabled with `ros2 param set /wireless_yolo_sensor display false`. WSLg or another GUI display is required. Camera reads run in a background thread; the default read timeout is 5 seconds to tolerate delayed frames before reconnecting. `Connected to wireless camera stream` only confirms the stream opened, while `Camera preview window opened` confirms a frame reached OpenCV. If no frame arrives by the configured read timeout, the node logs `Stream opened, but no camera frame has arrived yet`. Press `q` in the preview window to stop the sensor node; the controller's detection timeout returns steering and throttle to neutral.

## Serial and safety notes

The copied controller sends one newline-terminated `steering_voltage,throttle_voltage` pair per control tick at 115200 baud. The receiver source currently present in `car-drive-test/car_reciever.ino` is camera-server firmware rather than the described serial/DAC receiver, so confirm the actual receiver parser and voltage behavior before enabling control. The default PD gains and steering limit are conservative starting values, not vehicle-tuned calibration.

An empty detection message immediately returns outputs to neutral. If detections stop arriving, the 0.5-second target timeout also neutralizes outputs. A serial failure stops further command attempts; it cannot guarantee neutral if the physical link itself has failed.

On graceful control-node shutdown, the timer is canceled, one final neutral steering/throttle line is sent and flushed, then the serial port is closed. A forced process kill or failed serial link cannot guarantee command delivery.

## Tests

```bash
colcon test --packages-select auto_car_control
colcon test-result --verbose
```