# ESP32-S3 Camera Serial Streaming & CV Pipeline Setup

This repository contains the configuration, Arduino firmware, and Python script to stream live camera frames from a Freenove ESP32-S3 CAM to a host PC over a single USB cable for computer vision processing.

---

## 1. Overview & Setup

### Architecture

1. **Data Acquisition:** The ESP32-S3 captures frame buffers in JPEG format using its onboard camera sensor and stores them in 8MB OPI PSRAM.
2. **Serial Transmission:** The ESP32 packages each frame with a custom 6-byte header and streams raw bytes over USB Serial at 115200 baud.
3. **Data Decoding:** A Python script reads the serial byte stream using `pyserial`, synchronizes on the frame header, extracts the JPEG payload, and decodes it into an OpenCV frame matrix for real-time visualization and tracking.

### Recommended Arduino IDE Settings

* **Board:** `ESP32S3 Dev Module`
* **USB CDC On Boot:** `Enabled`
* **Upload Mode:** `UART0 / Hardware CDC`
* **PSRAM:** `OPI PSRAM`

---

## 2. Firmware Implementation (`camera_stream.ino`)

Upload this sketch to the ESP32-S3 board. Ensure the Arduino Serial Monitor is closed before launching any Python client.

```cpp
#include "esp_camera.h"
#include "Arduino.h"

// Freenove ESP32-S3 CAM Pin Definitions
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM    4
#define SIOC_GPIO_NUM    5

#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM    13

void setup() {
  Serial.begin(115200);

  // Pause briefly for USB CDC connection stabilization
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 3000)) {
    delay(10);
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;            // 1-63 range
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed: 0x%x\n", err);
    return;
  }
}

void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  // 1. Send 2-Byte Sync Marker
  Serial.write(0xAA);
  Serial.write(0xBB);

  // 2. Send 4-Byte Payload Size (Little-Endian)
  uint32_t len = fb->len;
  Serial.write((uint8_t*)&len, 4);

  // 3. Send JPEG Buffer Payload
  Serial.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

```

---

## 3. Host Receiver Script (`camera_viewer.py`)

Run this Python script on the laptop to capture, parse, and display the stream.

```python
import serial
import cv2
import numpy as np

# Adjust PORT to match system Device Manager / list_ports output
PORT = 'COM4' 
BAUD = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"Connected to {PORT} at {BAUD} baud.")
    except serial.SerialException as e:
        print(f"Error opening port {PORT}: {e}")
        return

    def read_frame():
        while True:
            # Sync on header sequence: 0xAA 0xBB
            if ser.read(1) == b'\xAA':
                if ser.read(1) == b'\xBB':
                    # Read 4-byte payload size
                    length_bytes = ser.read(4)
                    if len(length_bytes) < 4:
                        continue
                    length = int.from_bytes(length_bytes, byteorder='little')
                    
                    # Read binary JPEG payload
                    jpg_data = ser.read(length)
                    if len(jpg_data) == length:
                        # Convert byte array to numpy array and decode image
                        img_np = np.frombuffer(jpg_data, dtype=np.uint8)
                        return cv2.imdecode(img_np, cv2.IMREAD_COLOR)

    try:
        while True:
            frame = read_frame()
            if frame is not None:
                cv2.imshow("ESP32-S3 Stream", frame)

            # Exit cleanly when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        ser.close()
        cv2.destroyAllWindows()
        print("Serial port closed and windows destroyed.")

if __name__ == "__main__":
    main()

```

---

## 4. How It Works

### Packet Framing Protocol

Because serial communication is an uninterrupted stream of raw bytes, the receiver requires a deterministic protocol to identify where a JPEG frame starts and ends.

```
+-------------------+----------------------+-----------------------------+
| Header (2 Bytes)  | Length Field (4 B)   | Payload Data (N Bytes)      |
+-------------------+----------------------+-----------------------------+
|    0xAA  0xBB     |  uint32_t (4-byte)   | Compressed JPEG Byte Buffer |
+-------------------+----------------------+-----------------------------+

```

1. **Header Identification (`0xAA 0xBB`):** The Python script scans incoming bytes one at a time. It ignores garbage data until it detects the consecutive byte sequence `0xAA` followed by `0xBB`.
2. **Payload Size Reading:** Immediately following the header, the next 4 bytes are converted to an integer representing the exact byte size ($N$) of the compressed frame.
3. **Payload Extraction & Decoding:** The script blocks until exactly $N$ bytes are read from the buffer, wrapping them into a 1D NumPy array. `cv2.imdecode()` parses the JPEG byte buffer directly in memory into an $H \times W \times 3$ matrix ready for Computer Vision operations.

### Port Management

* Windows limits access to COM ports to a single active handle. `PermissionError: Access is denied` indicates that another application (such as the Arduino Serial Monitor or an orphaned background Python process) is maintaining an open handle on the interface.