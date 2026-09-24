import cv2

# Explicitly specify port 81 and the /stream endpoint
stream_url = "http://192.168.4.1:81/stream"

def main():
    print(f"Connecting to {stream_url}...")
    cap = cv2.VideoCapture(stream_url)

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