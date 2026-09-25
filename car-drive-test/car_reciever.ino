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
const char* password = "00000000"; // Must be at least 8 characters

httpd_handle_t stream_httpd = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// HTTP handler for continuous video streaming
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

  // Initialize Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maximize Wi-Fi range
  
  startCameraServer();
  Serial.println("\nCamera Stream Ready!");
  Serial.print("Connect to Wi-Fi SSID: ");
  Serial.println(ssid);
  Serial.print("Stream URL: http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
}

void loop() {
  delay(10000);
}