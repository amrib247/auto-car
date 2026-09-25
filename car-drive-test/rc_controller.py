import serial
import time

class RCController:
    def __init__(self, port, baudrate=115200):
        # Open serial connection to the ESP32
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        
        # Steering Voltage Parameters
        self.steer_min = 0.0          # Max left
        self.steer_dz_lower = 1.56    # Edge of left deadzone
        self.steer_rest = 1.56        # True center
        self.steer_dz_upper = 1.56    # Edge of right deadzone
        self.steer_max = 3.1         # Max right
        
        # Throttle Voltage Parameters
        self.throttle_min = 0.0       # Max reverse / brake
        self.throttle_dz_lower = 1.37 # Edge of reverse deadzone
        self.throttle_dz_upper = 2.16 # Edge of forward deadzone
        self.throttle_rest = (self.throttle_dz_lower + self.throttle_dz_upper) / 2    # True neutral
        self.throttle_max = 3.22      # Max forward
        
        # Current State Tracking
        self.current_steer_v = self.steer_rest
        self.current_throttle_v = self.throttle_rest
        self.last_throttle_intensity = 0.0
        
    def set_steering(self, intensity):
        """Scale -1.0 (left) to 1.0 (right) into steering voltage with deadzones."""
        intensity = max(-1.0, min(1.0, intensity))  # Clamp between -1 and 1
        
        if intensity == 0:
            self.current_steer_v = self.steer_rest
        elif intensity > 0:
            # Map 0 to 1.0 between the upper deadzone and max voltage
            self.current_steer_v = self.steer_dz_upper + (intensity * (self.steer_max - self.steer_dz_upper))
        else:
            # Map 0 to -1.0 between the lower deadzone and min voltage
            self.current_steer_v = self.steer_dz_lower - (abs(intensity) * (self.steer_dz_lower - self.steer_min))
            
        self._send_command()

    def set_throttle(self, intensity):
        """Scale -1.0 (reverse) to 1.0 (forward) into throttle voltage with auto-double-tap."""
        intensity = max(-1.0, min(1.0, intensity))  # Clamp between -1 and 1
        
        # Auto-Double-Click Logic for Reverse
        if intensity < 0 and self.last_throttle_intensity >= 0:
            # Step 1: Send momentary brake
            self.current_throttle_v = self.throttle_min
            self._send_command()
            time.sleep(0.05)
            
            # Step 2: Return to neutral
            self.current_throttle_v = self.throttle_rest
            self._send_command()
            time.sleep(0.05)
            
            # Step 3: Proceed to calculate and send the actual reverse command
            
        self.last_throttle_intensity = intensity
        
        if intensity == 0:
            self.current_throttle_v = self.throttle_rest
        elif intensity > 0:
            # Map 0 to 1.0 between the upper deadzone and max voltage
            self.current_throttle_v = self.throttle_dz_upper + (intensity * (self.throttle_max - self.throttle_dz_upper))
        else:
            # Map 0 to -1.0 between the lower deadzone and min voltage
            self.current_throttle_v = self.throttle_dz_lower - (abs(intensity) * (self.throttle_dz_lower - self.throttle_min))
            
        self._send_command()
        
    def _send_command(self):
        """Send the formatted string: steer_voltage,throttle_voltage\\n"""
        command = f"{self.current_steer_v:.3f},{self.current_throttle_v:.3f}\n"
        self.ser.write(command.encode('utf-8'))
        
    def close(self):
        """Return to neutral rest voltages and close the serial port."""
        self.set_steering(0.0)
        self.set_throttle(0.0)
        self.ser.close()