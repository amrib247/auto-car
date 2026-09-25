import serial
import time

def main():
    PORT = 'COM3'  # Update if necessary
    BAUDRATE = 115200
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print(f"Connected to ESP32 on {PORT}.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Starting neutral voltages
    steer_v = 1.56
    throttle_v = (1.37 + 2.17) / 2
    last_throttle_v = throttle_v
    step_size = 0.05

    print("\n--- Calibration Mode ---")
    print("W / S : Increase / Decrease Throttle")
    print("D / A : Increase / Decrease Steering")
    print("+ / - : Increase / Decrease step size (currently 0.05V)")
    print("R     : Reset to neutral (1.61V)")
    print("Q     : Quit")
    print("Press the key, then press Enter.\n")

    try:
        while True:
            # Auto-Double-Click Logic for Reverse Calibration
            if throttle_v < 1.61 and last_throttle_v >= 1.61:
                print(">>> Auto-engaging reverse (Brake -> Neutral)...")
                # Send momentary full brake
                ser.write(f"{steer_v:.3f},0.000\n".encode('utf-8'))
                time.sleep(0.05)
                # Return to neutral
                ser.write(f"{steer_v:.3f},1.610\n".encode('utf-8'))
                time.sleep(0.05)
                
            # Send the target voltage state
            command = f"{steer_v:.3f},{throttle_v:.3f}\n"
            ser.write(command.encode('utf-8'))
            last_throttle_v = throttle_v
            
            print(f"Current Output -> Steer: {steer_v:.3f}V | Throttle: {throttle_v:.3f}V | Step: {step_size:.3f}V")
            
            user_input = input("Action: ").strip().lower()

            if user_input == 'q':
                break
            elif user_input == 'r':
                steer_v = 1.61
                throttle_v = 1.61
                last_throttle_v = 1.61
            elif user_input == 'w':
                throttle_v = min(3.3, throttle_v + step_size)
            elif user_input == 's':
                throttle_v = max(0.0, throttle_v - step_size)
            elif user_input == 'd':
                steer_v = min(3.3, steer_v + step_size)
            elif user_input == 'a':
                steer_v = max(0.0, steer_v - step_size)
            elif user_input == '+':
                step_size = min(0.5, step_size + 0.01)
            elif user_input == '-':
                step_size = max(0.01, step_size - 0.01)
            else:
                print("Invalid command.")
                
    except KeyboardInterrupt:
        pass
    finally:
        print("\nReturning to neutral and closing...")
        ser.write(b"1.610,1.610\n")
        ser.close()

if __name__ == "__main__":
    main()