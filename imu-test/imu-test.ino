#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "ICM_20948.h"

// Custom I2C pins for ESP32-S3-CAM
#define SDA_PIN 43
#define SCL_PIN 44
#define AD0_VAL 0 // I2C address 0x68

// Wi-Fi Access Point Credentials
const char* ap_ssid = "ESP32_CAM_RC";
const char* ap_password = "00000000"; // Minimum 8 characters

// Subnet broadcast IP for default ESP32 AP range (192.168.4.x)
const char* broadcast_ip = "192.168.4.255"; 
const int udp_port = 12345;

WiFiUDP udp;
ICM_20948_I2C myICM; 

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); // Configure GPIO 43 (SDA) and GPIO 44 (SCL)

  // Configure ESP32 as a Wi-Fi Access Point
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("Wi-Fi Access Point started. Server IP: ");
  Serial.println(apIP); // Default is 192.168.4.1

  // Initialize ICM-20948
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Connecting to ICM-20948...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  Serial.println("ICM-20948 connected and transmitting!");
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
    
    // Format payload: accX,accY,accZ,gyrX,gyrY,gyrZ
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             myICM.accX(), myICM.accY(), myICM.accZ(),
             myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
             
    // Broadcast payload over UDP on the local AP network
    udp.beginPacket(broadcast_ip, udp_port);
    udp.print(buffer);
    udp.endPacket();
  }
  delay(20); // ~50Hz transmission rate
}