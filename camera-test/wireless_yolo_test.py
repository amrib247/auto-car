import cv2
from ultralytics import YOLO

# Load the YOLOv8 nano model (downloads automatically the first time)
model = YOLO('yolov8n.pt')

STREAM_URL = "http://192.168.4.1:81/stream"

def main():
    print(f"Connecting to {STREAM_URL}...")
    cap = cv2.VideoCapture(STREAM_URL)

    if not cap.isOpened():
        print("Could not open stream. Verify laptop is connected to ESP32-CAM-RC.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream dropped.")
            break

        # Run YOLOv8 inference on the frame
        # conf=0.5 ignores predictions with under 50% confidence
        results = model(frame, conf=0.5, verbose=False)

        # Draw the results on the frame
        annotated_frame = results[0].plot()

        # Extract coordinates for RC control logic
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get coordinates for the bounding box
                x1, y1, x2, y2 = box.xyxy[0] 
                
                # Calculate the center point of the object
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                # Get the class name (e.g., 'person', 'sports ball')
                class_id = int(box.cls[0])
                class_name = model.names[class_id]

                # Draw a target reticle at the center
                cv2.drawMarker(annotated_frame, (center_x, center_y), (0, 0, 255), 
                               cv2.MARKER_CROSS, 20, 2)
                
                # In the future, center_x will determine if you steer LEFT or RIGHT
                # center_y (or box size) will determine THROTTLE (distance to object)

        cv2.imshow("RC Vision - YOLOv8", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()