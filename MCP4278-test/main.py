import time
import board
import busio
import adafruit_mcp4728

# Initialize I2C on SCL (A20) and SDA (A22)
i2c = busio.I2C(board.SCL, board.SDA) 
mcp4728 = adafruit_mcp4728.MCP4728(i2c)

while True:
    # Output 0 V (Code 0)
    mcp4728.channel_a.raw_value = 0
    mcp4728.channel_b.raw_value = 0
    time.sleep(10)

    # Output roughly 1.65 V (Code 2048)
    mcp4728.channel_a.raw_value = 2048
    mcp4728.channel_b.raw_value = 2048
    time.sleep(10)

    # Output ~3.3 V (Code 4095)
    mcp4728.channel_a.raw_value = 4095
    mcp4728.channel_b.raw_value = 4095
    time.sleep(10)