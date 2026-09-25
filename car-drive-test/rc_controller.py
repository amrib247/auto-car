import serial

class RCController:
    def __init__(self, port, baudrate=115200):
        # Open serial connection to the ESP32
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        
        # Steering Voltage Baselines
        self.min_steer = 0.0
        self.rest_steer = 1.61
        self.max_steer = 3.22
        
        # Throttle Voltage Baselines
        self.min_throttle = 0.0
        self.rest_throttle = 1.61
        self.max_throttle = 3.22
        
        # Current State Tracking
        self.current_steer_v = self.rest_steer
        self.current_throttle_v = self.rest_throttle
        
    def set_steering(self, intensity):
        """Scale -1.0 (left) to 1.0 (right) into steering voltage."""
        intensity = max(-1.0, min(1.0, intensity))  # Clamp between -1 and 1
        
        if intensity >= 0:
            self.current_steer_v = self.rest_steer + (intensity * (self.max_steer - self.rest_steer))
        else:
            self.current_steer_v = self.rest_steer + (intensity * (self.rest_steer - self.min_steer))
            
        self._send_command()

    def set_throttle(self, intensity):
        """Scale -1.0 (reverse) to 1.0 (forward) into throttle voltage."""
        intensity = max(-1.0, min(1.0, intensity))  # Clamp between -1 and 1
        
        if intensity >= 0:
            self.current_throttle_v = self.rest_throttle + (intensity * (self.max_throttle - self.rest_throttle))
        else:
            self.current_throttle_v = self.rest_throttle + (intensity * (self.rest_throttle - self.min_throttle))
            
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