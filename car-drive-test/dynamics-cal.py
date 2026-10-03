import time
import keyboard
import numpy as np
from scipy.optimize import minimize
from rc_controller import RCController

# --- Optimization Constants ---
WHEELBASE = 0.17 # L: Wheelbase in meters (UPDATE THIS to match your car)
TARGET_THETA = 2 * np.pi # Assuming exactly one counter-clockwise loop (use -2*pi for clockwise)
W_THETA = 1.0 # Weight multiplier for heading error

def simulate_trajectory(cv, cs, dt_log, s_log, t_log):
    """Integrates the kinematic bicycle model over the logged inputs."""
    x, y, theta = 0.0, 0.0, 0.0
    
    for dt, s, t in zip(dt_log, s_log, t_log):
        v = cv * t
        delta = cs * s
        
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += (v / WHEELBASE) * np.tan(delta) * dt
        
    return x, y, theta

def cost_function(params, dt_log, s_log, t_log):
    """Calculates the error between final integrated state and the true (0,0) starting state."""
    cv, cs = params
    x_final, y_final, theta_final = simulate_trajectory(cv, cs, dt_log, s_log, t_log)
    
    # Error is the squared distance from origin + squared heading error
    pos_error = x_final**2 + y_final**2
    heading_error = W_THETA * (theta_final - TARGET_THETA)**2
    
    return pos_error + heading_error

def main():
    # Update this to match your ESP32's COM port
    PORT = 'COM3' 
    
    # --- Data Logging Arrays ---
    dt_log = []
    s_log = []
    t_log = []
    
    try:
        car = RCController(port=PORT)
        print(f"Connected to ESP32 on {PORT}.")
        print("Drive ONE complete counter-clockwise loop, ending exactly where you started.")
        print("Control the car using W, A, S, D. Press 'ESC' to quit and optimize.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    try:
        last_time = time.time()
        
        while True:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Exit condition
            if keyboard.is_pressed('esc'):
                print("\nExiting and returning to neutral...")
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
            
            # Log the inputs for optimization
            dt_log.append(dt)
            s_log.append(steer_intensity)
            t_log.append(throttle_intensity)
            
            # 50ms delay to prevent flooding the ESP32 serial buffer (20 updates per second)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        car.close()
        
    # --- Run System Identification Optimization ---
    if len(dt_log) > 0:
        print("\nRunning optimization to find cv and cs...")
        
        # Initial guesses: cv = 1.0 m/s, cs = 0.5 rad (~28 degrees)
        initial_guess = [1.0, 0.5]
        
        # Setup bounds to ensure realistic physical limits 
        # cv > 0.1 m/s, cs between 0.1 rad and 1.5 rad (approx 85 deg)
        bounds = [(0.1, 10.0), (0.1, 1.5)]
        
        result = minimize(
            cost_function, 
            initial_guess, 
            args=(dt_log, s_log, t_log), 
            bounds=bounds,
            method='L-BFGS-B'
        )
        
        cv_opt, cs_opt = result.x
        x_final, y_final, theta_final = simulate_trajectory(cv_opt, cs_opt, dt_log, s_log, t_log)
        
        print("\n=== Optimization Results ===")
        print(f"Success: {result.success}")
        print(f"Optimal Velocity Scaling (cv): {cv_opt:.3f} m/s")
        print(f"Optimal Steering Scaling (cs): {cs_opt:.3f} rad ({np.degrees(cs_opt):.1f} deg)")
        print("\nSimulated Final State using these parameters:")
        print(f"X: {x_final:.3f}m, Y: {y_final:.3f}m, Heading: {np.degrees(theta_final):.1f} deg")

if __name__ == "__main__":
    main()