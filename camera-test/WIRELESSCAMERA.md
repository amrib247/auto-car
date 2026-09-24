# ESP32-S3 Wireless Camera & YOLO Vision Pipeline Setup

This document details the transition from the wired serial stream to a wireless Access Point (SoftAP) MJPEG camera stream, along with real-time YOLOv8 object detection accelerated by PyTorch CUDA on the host PC.

---

## 1. System Overview & Wireless Architecture

To untether the ESP32-S3 CAM from the laptop, the camera subsystem was migrated from raw serial byte streaming to a Wi-Fi Access Point HTTP server.

```
[ ESP32-S3 CAM ] (SoftAP: ESP32-CAM-RC)
       |
  (Wi-Fi 2.4 GHz / HTTP MJPEG Stream on Port 81)
       v
[ Laptop PC ] 
  ├── Wi-Fi Interface (Connected to 192.168.4.1)
  ├── FreshFrameReader (Threaded Buffer Flush)
  ├── PyTorch / CUDA (GPU Hardware Acceleration)
  └── YOLOv8 Nano Inference -> Bounding Boxes & Centroid Coordinates (x, y)

```

* **Network Mode:** Access Point (SoftAP) broadcasting SSID `ESP32-CAM-RC` (IP: `192.168.4.1`).
* **Protocol:** HTTP MJPEG stream served on port `81` at route `/stream`.
* **Powering:** The ESP32-S3 can be powered via any 5V USB source (laptop USB port for desktop testing, or chassis-mounted USB power bank for untethered operation).

---

## 2. Firmware Implementation (`camera_wireless.ino`)

This sketch configures the camera pins for the Freenove board, initializes OPI PSRAM, starts the Wi-Fi Access Point, and hosts the HTTP MJPEG stream server.

```cpp
#include "esp_camera.h"
#include "WiFi.h"
#include "esp_http_server.h"

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

const char* ssid = "ESP32-CAM-RC";
const char* password = "password123";

httpd_handle_t stream_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    
    _jpg_buf_len = fb->len;
    _jpg_buf = fb->buf;

    if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    
    size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
    if (httpd_resp_send_chunk(req, (const char *)part_buf, hlen) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    
    if (httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }
    
    esp_camera_fb_return(fb);
  }
  return res;
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);

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
  config.frame_size = FRAMESIZE_QVGA; 
  config.jpeg_quality = 12;            
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  startCameraServer();
  Serial.println("\nCamera Stream Ready!");
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
}

void loop() {
  delay(10000);
}

```

---

## 3. Wireless Stream Receiver (`wireless_camera.py`)

This basic receiver script connects over Wi-Fi using standard OpenCV `VideoCapture` to verify network connectivity and stream decoding before adding AI processing.

```python
import cv2

STREAM_URL = "http://192.168.4.1:81/stream"

def main():
    print(f"Connecting to {STREAM_URL}...")
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print("Could not open stream. Verify laptop is connected to ESP32-CAM-RC Wi-Fi.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame drop or stream interrupted.")
            break

        cv2.imshow("ESP32-S3 Wireless Stream", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

```

---

## 4. Real-Time Object Detection & Performance Pipeline (`wireless_yolo_test.py`)

This script integrates YOLOv8 Nano for real-time object detection and centroid tracking while addressing hardware acceleration and network stream latency.

### Key Optimization Principles

1. **GPU Acceleration (CUDA):** Forced inference onto Nvidia CUDA (`device='cuda'`) with FP16 half-precision (`half=True`) to minimize frame processing times.
2. **Threaded Buffer Flushing (`FreshFrameReader`):** OpenCV's default `VideoCapture` buffers incoming network frames in RAM. When inference takes longer than frame ingestion, buffer buildup causes severe video delay (2–5 seconds). The custom reader thread runs continuously in the background, throwing away stale frames so the main loop always evaluates the absolute newest frame.

```python
import cv2
import torch
import threading
from ultralytics import YOLO

# 1. Device Selection & Model Initialization
device = 'cuda' if torch.cuda.is_available() else 'cpu'
print(f"Running inference on: {device.upper()}")

model = YOLO('yolov8n.pt').to(device)

STREAM_URL = "http://192.168.4.1:81/stream"

# 2. Threaded Reader to Eliminate Network Buffer Latency
class FreshFrameReader:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret = False
        self.frame = None
        self.running = True
        
        if self.cap.isOpened():
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.ret = ret
                self.frame = frame

    def read(self):
        return self.ret, self.frame

    def release(self):
        self.running = False
        self.cap.release()

def main():
    print(f"Connecting to stream at {STREAM_URL}...")
    stream = FreshFrameReader(STREAM_URL)

    # Wait for first frame
    while stream.frame is None:
        pass

    print("Stream connected! Press 'q' to quit.")

    while True:
        ret, frame = stream.read()
        if not ret or frame is None:
            continue

        # 3. GPU Inference with Half-Precision (FP16)
        results = model(frame, device=device, half=(device == 'cuda'), conf=0.5, verbose=False)

        annotated_frame = results[0].plot()

        # 4. Target Centroid Extraction for Vehicle Control Loop
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Visual Target Reticle
                cv2.drawMarker(annotated_frame, (center_x, center_y), (0, 0, 255), 
                               cv2.MARKER_CROSS, 20, 2)

        cv2.imshow("RC Vision - Low Latency YOLO", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

```

---

## 5. Troubleshooting & Configuration Lessons

* **Port Mismatch:** The ESP32 web server hosts the MJPEG stream on port `81` (`/stream`), not the standard HTTP port `80`. Ensure URLs explicitly request `[http://192.168.4.1:81/stream](http://192.168.4.1:81/stream)`.
* **PyTorch CUDA Installation:** If PyTorch defaults to `CPU`, force-reinstall CUDA binaries using:
```powershell
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121 --force-reinstall

```


* **Network Isolation:** While connected to `ESP32-CAM-RC`, the host laptop will lose general internet access. This is expected behavior for SoftAP mode.