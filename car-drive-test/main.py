import time
import keyboard
from rc_controller import RCController

def main():
    # Update this to match your ESP32's COM port (e.g., 'COM3' for Windows, '/dev/ttyUSB0' for Mac/Linux)
    PORT = 'COM3' 
    
    try:
        car = RCController(port=PORT)
        print(f"Connected to ESP32 on {PORT}.")
        print("Control the car using W, A, S, D. Press 'ESC' to quit.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    try:
        while True:
            # Exit condition
            if keyboard.is_pressed('esc'):
                print("Exiting and returning to neutral...")
                break
                
            # Determine Steering Intensity (-1.0 to 1.0)
            steer_intensity = 0.0
            if keyboard.is_pressed('a'):
                steer_intensity -= 1.0
            if keyboard.is_pressed('d'):
                steer_intensity += 1.0
                
            # Determine Throttle Intensity (-1.0 to 1.0)
            throttle_intensity = 0.0
            if keyboard.is_pressed('s'):
                throttle_intensity -= 1.0
            if keyboard.is_pressed('w'):
                throttle_intensity += 1.0
                
            # Update the controller class
            car.set_steering(steer_intensity)
            car.set_throttle(throttle_intensity)
            
            # 50ms delay to prevent flooding the ESP32 serial buffer (20 updates per second)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        car.close()

if __name__ == "__main__":
    main()