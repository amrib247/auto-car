#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "esp_camera.h"
#include "esp_http_server.h"

// Freenove ESP32-S3 CAM pin definitions
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

// ICM-20948 I2C pins
#define SDA_PIN 43
#define SCL_PIN 44
#define AD0_VAL 0

// Set these to the home Wi-Fi network and the laptop's IPv4 address.
const char* WIFI_SSID = "home wifi ssid";
const char* WIFI_PASSWORD = "password";
const IPAddress LAPTOP_IP(192, 168, 1, 50);
const uint16_t IMU_UDP_PORT = 12345;

static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
static const uint32_t IMU_RETRY_INTERVAL_MS = 2000;
static const uint32_t IMU_SEND_INTERVAL_MS = 20;
#define STREAM_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;

WiFiUDP udp;
ICM_20948_I2C imu;
httpd_handle_t streamHttpd = nullptr;

bool imuReady = false;
bool wifiWasConnected = false;
bool udpReady = false;
uint32_t nextWifiAttempt = 0;
uint32_t nextUdpAttempt = 0;
uint32_t nextImuAttempt = 0;
uint32_t lastImuSend = 0;
uint32_t lastUdpError = 0;

static esp_err_t streamHandler(httpd_req_t* request) {
  char partHeader[64];
  if (httpd_resp_set_type(request, STREAM_CONTENT_TYPE) != ESP_OK) {
    return ESP_FAIL;
  }

  while (true) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame == nullptr) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    static const char boundary[] = "\r\n--" STREAM_BOUNDARY "\r\n";

    esp_err_t result = httpd_resp_send_chunk(
        request, boundary, sizeof(boundary) - 1);
    if (result == ESP_OK) {
      const int headerLength = snprintf(
          partHeader, sizeof(partHeader),
          "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
          static_cast<unsigned int>(frame->len));
      result = httpd_resp_send_chunk(request, partHeader, headerLength);
    }
    if (result == ESP_OK) {
      result = httpd_resp_send_chunk(
          request, reinterpret_cast<const char*>(frame->buf), frame->len);
    }

    esp_camera_fb_return(frame);
    if (result != ESP_OK) {
      return result;
    }
  }
}

bool startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;

  httpd_uri_t streamUri = {};
  streamUri.uri = "/stream";
  streamUri.method = HTTP_GET;
  streamUri.handler = streamHandler;

  esp_err_t result = httpd_start(&streamHttpd, &config);
  if (result != ESP_OK) {
    Serial.printf("Camera HTTP server failed to start: 0x%x\n", result);
    return false;
  }

  result = httpd_register_uri_handler(streamHttpd, &streamUri);
  if (result != ESP_OK) {
    Serial.printf("Camera stream route registration failed: 0x%x\n", result);
    httpd_stop(streamHttpd);
    streamHttpd = nullptr;
    return false;
  }

  return true;
}

bool initializeImu() {
  imu.begin(Wire, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.printf("ICM-20948 initialization failed: %s\n",
                  imu.statusString());
    return false;
  }

  Serial.println("ICM-20948 connected");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  camera_config_t cameraConfig = {};
  cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  cameraConfig.ledc_timer = LEDC_TIMER_0;
  cameraConfig.pin_d0 = Y2_GPIO_NUM;
  cameraConfig.pin_d1 = Y3_GPIO_NUM;
  cameraConfig.pin_d2 = Y4_GPIO_NUM;
  cameraConfig.pin_d3 = Y5_GPIO_NUM;
  cameraConfig.pin_d4 = Y6_GPIO_NUM;
  cameraConfig.pin_d5 = Y7_GPIO_NUM;
  cameraConfig.pin_d6 = Y8_GPIO_NUM;
  cameraConfig.pin_d7 = Y9_GPIO_NUM;
  cameraConfig.pin_xclk = XCLK_GPIO_NUM;
  cameraConfig.pin_pclk = PCLK_GPIO_NUM;
  cameraConfig.pin_vsync = VSYNC_GPIO_NUM;
  cameraConfig.pin_href = HREF_GPIO_NUM;
  cameraConfig.pin_sccb_sda = SIOD_GPIO_NUM;
  cameraConfig.pin_sccb_scl = SIOC_GPIO_NUM;
  cameraConfig.pin_pwdn = PWDN_GPIO_NUM;
  cameraConfig.pin_reset = RESET_GPIO_NUM;
  cameraConfig.xclk_freq_hz = 20000000;
  cameraConfig.pixel_format = PIXFORMAT_JPEG;
  cameraConfig.frame_size = FRAMESIZE_QVGA;
  cameraConfig.jpeg_quality = 12;
  cameraConfig.fb_count = 2;
  cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;

  const esp_err_t cameraResult = esp_camera_init(&cameraConfig);
  if (cameraResult != ESP_OK) {
    Serial.printf("Camera initialization failed: 0x%x\n", cameraResult);
    while (true) {
      delay(1000);
    }
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  imuReady = initializeImu();
  nextImuAttempt = millis() + IMU_RETRY_INTERVAL_MS;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  nextWifiAttempt = millis() + WIFI_RETRY_INTERVAL_MS;
  Serial.println("Connecting to home Wi-Fi...");

  if (startCameraServer()) {
    Serial.println("Camera HTTP server started on port 81");
  }
}

void loop() {
  const uint32_t now = millis();
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;

  if (wifiConnected && !wifiWasConnected) {
    Serial.print("Connected to Wi-Fi. ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Camera stream: http://%s:81/stream\n",
                  WiFi.localIP().toString().c_str());
    Serial.printf("Sending IMU UDP data to %s:%u\n",
                  LAPTOP_IP.toString().c_str(), IMU_UDP_PORT);
  } else if (!wifiConnected && wifiWasConnected) {
    Serial.println("Wi-Fi connection lost; waiting to reconnect");
    udp.stop();
    udpReady = false;
  }
  wifiWasConnected = wifiConnected;

  if (!wifiConnected && static_cast<int32_t>(now - nextWifiAttempt) >= 0) {
    Serial.println("Retrying home Wi-Fi connection...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    nextWifiAttempt = now + WIFI_RETRY_INTERVAL_MS;
  }

  if (wifiConnected && !udpReady &&
      static_cast<int32_t>(now - nextUdpAttempt) >= 0) {
    udpReady = udp.begin(IMU_UDP_PORT) == 1;
    if (!udpReady) {
      Serial.println("Failed to initialize the IMU UDP socket; will retry");
    }
    nextUdpAttempt = now + WIFI_RETRY_INTERVAL_MS;
  }

  if (!imuReady && static_cast<int32_t>(now - nextImuAttempt) >= 0) {
    imuReady = initializeImu();
    nextImuAttempt = now + IMU_RETRY_INTERVAL_MS;
  }

  if (imuReady && wifiConnected && udpReady &&
      static_cast<int32_t>(now - lastImuSend) >=
          static_cast<int32_t>(IMU_SEND_INTERVAL_MS) &&
      imu.dataReady()) {
    imu.getAGMT();

    char payload[128];
    const int payloadLength = snprintf(
        payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        imu.accX(), imu.accY(), imu.accZ(),
        imu.gyrX(), imu.gyrY(), imu.gyrZ());

    bool sent = false;
    if (payloadLength > 0 &&
        payloadLength < static_cast<int>(sizeof(payload)) &&
        udp.beginPacket(LAPTOP_IP, IMU_UDP_PORT) == 1) {
      const size_t written = udp.print(payload);
      const bool packetSent = udp.endPacket() == 1;
      sent = written == static_cast<size_t>(payloadLength) && packetSent;
    }

    if (!sent && static_cast<int32_t>(now - lastUdpError) >= 5000) {
      Serial.println("Failed to send IMU UDP packet");
      lastUdpError = now;
    }
    lastImuSend = now;
  }

  delay(2);
}
