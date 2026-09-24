import serial
import cv2
import numpy as np

# Change 'COM3' to match your board's COM port in Device Manager
PORT = 'COM4' 
BAUD = 921600

ser = serial.Serial(PORT, BAUD, timeout=1)

def read_frame():
    while True:
        # Scan for header bytes (0xAA 0xBB)
        if ser.read(1) == b'\xAA':
            if ser.read(1) == b'\xBB':
                # Read 4-byte payload length
                length_bytes = ser.read(4)
                if len(length_bytes) < 4:
                    continue
                length = int.from_bytes(length_bytes, byteorder='little')
                
                # Read raw JPEG buffer
                jpg_data = ser.read(length)
                if len(jpg_data) == length:
                    # Decode into OpenCV frame
                    img_np = np.frombuffer(jpg_data, dtype=np.uint8)
                    return cv2.imdecode(img_np, cv2.IMREAD_COLOR)

while True:
    frame = read_frame()
    if frame is not None:
        cv2.imshow("Freenove ESP32-S3 Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
cv2.destroyAllWindows()