#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "ICM_20948.h"

#define SDA_PIN 43
#define SCL_PIN 44
#define AD0_VAL 0 

// Enter your actual home Wi-Fi credentials here
const char* ssid = "home wifi ssid";
const char* password = "password";

// The IP address of your laptop on your home network
// (Find this using 'ipconfig' in command prompt, e.g., 192.168.1.50)
const char* target_ip = "your laptop ip"; 
const int udp_port = 12345;

WiFiUDP udp;
ICM_20948_I2C myICM; 

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); 

  // Connect to Home Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting to Home Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected! ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

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
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             myICM.accX(), myICM.accY(), myICM.accZ(),
             myICM.gyrX(), myICM.gyrY(), myICM.gyrZ());
             
    // Send directly to the laptop's IP address
    udp.beginPacket(target_ip, udp_port);
    udp.print(buffer);
    udp.endPacket();
  }
  delay(20); 
}