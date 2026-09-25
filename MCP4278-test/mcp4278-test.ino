#include "Adafruit_MCP4728.h"
#include "Wire.h"

Adafruit_MCP4728 mcp;

void setup(void) {
  Serial.begin(115200);
  
  // Wait for serial monitor to open
  while (!Serial) {
    delay(10); 
  }

  Serial.println("MCP4728 Bench Test - HBX 18859");

  // Initialize the MCP4728
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip. Check I2C wiring!");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  // Output: 0 V
  // Writing to Channel A (Steering) and Channel B (Throttle)
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0);
  Serial.println("Voltage: 0 V (Code 0)");
  delay(3000);

  // Output: ~1.65 V
  // Code 2048 is the approximate center for 3.3V logic
  mcp.setChannelValue(MCP4728_CHANNEL_A, 2048);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 2048);
  Serial.println("Voltage: ~1.65 V (Code 2048)");
  delay(3000);

  // Output: ~3.3 V
  // Code 4095 is the maximum value for a 12-bit DAC
  mcp.setChannelValue(MCP4728_CHANNEL_A, 4095);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 4095);
  Serial.println("Voltage: ~3.3 V (Code 4095)");
  delay(3000);
}