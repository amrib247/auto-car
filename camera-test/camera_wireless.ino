import socket
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# UDP Configuration
UDP_IP = "0.0.0.0" 
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# State Variables
max_points = 100
times = []
roll_data, pitch_data, yaw_data = [], [], []
px_data, py_data, pz_data = [], [], []

# Filter & Integration states
roll, pitch, yaw = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0
px, py, pz = 0.0, 0.0, 0.0
last_time = time.time()

# Plot Setup
fig, (ax_orient, ax_pos) = plt.subplots(2, 1, figsize=(8, 6))
fig.tight_layout(pad=3.0)

line_r, = ax_orient.plot([], [], label='Roll (deg)')
line_p, = ax_orient.plot([], [], label='Pitch (deg)')
line_y, = ax_orient.plot([], [], label='Yaw (deg)')
ax_orient.set_title("Orientation (Pose)")
ax_orient.set_xlim(0, max_points)
ax_orient.set_ylim(-180, 180)
ax_orient.legend(loc="upper left")

line_px, = ax_pos.plot([], [], label='Pos X (m)')
line_py, = ax_pos.plot([], [], label='Pos Y (m)')
line_pz, = ax_pos.plot([], [], label='Pos Z (m)')
ax_pos.set_title("Position Estimate (Warning: Will drift rapidly!)")
ax_pos.set_xlim(0, max_points)
ax_pos.legend(loc="upper left")

def update(frame):
    global roll, pitch, yaw, vx, vy, vz, px, py, pz, last_time
    
    current_time = time.time()
    dt = current_time - last_time
    updated = False

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            decoded = data.decode('utf-8').strip()
            parts = decoded.split(',')
            
            if len(parts) >= 6:
                # 1. Parse Data: Accel (mg -> g), Gyro (deg/s -> rad/s)
                ax = float(parts[0]) / 1000.0
                ay = float(parts[1]) / 1000.0
                az = float(parts[2]) / 1000.0
                gx = math.radians(float(parts[3]))
                gy = math.radians(float(parts[4]))
                gz = math.radians(float(parts[5]))

                # 2. Orientation (Complementary Filter)
                # Calculate tilt from accelerometer
                roll_acc = math.atan2(ay, az)
                pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))

                # Fuse with Gyroscope (98% Gyro, 2% Accel correction)
                alpha = 0.98
                roll = alpha * (roll + gx * dt) + (1 - alpha) * roll_acc
                pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc
                yaw = yaw + gz * dt # Yaw drifts over time without Magnetometer

                # 3. Position Estimation (Gravity removal & Double Integration)
                # Simple gravity compensation assuming Z is up when flat
                # Convert g to m/s^2 (1g = 9.81 m/s^2)
                lin_ax = (ax + math.sin(pitch)) * 9.81
                lin_ay = (ay - math.sin(roll) * math.cos(pitch)) * 9.81
                lin_az = (az - math.cos(roll) * math.cos(pitch)) * 9.81 

                # Integrate Acceleration to Velocity
                vx += lin_ax * dt
                vy += lin_ay * dt
                vz += lin_az * dt

                # Apply aggressive dampening to velocity to prevent graph explosion (hack for IMU drift)
                vx *= 0.95; vy *= 0.95; vz *= 0.95

                # Integrate Velocity to Position
                px += vx * dt
                py += vy * dt
                pz += vz * dt

                # Store data for plotting
                roll_data.append(math.degrees(roll))
                pitch_data.append(math.degrees(pitch))
                yaw_data.append(math.degrees(yaw))
                px_data.append(px)
                py_data.append(py)
                pz_data.append(pz)

                if len(roll_data) > max_points:
                    roll_data.pop(0); pitch_data.pop(0); yaw_data.pop(0)
                    px_data.pop(0); py_data.pop(0); pz_data.pop(0)
                
                updated = True
        except BlockingIOError:
            break 
        except Exception as e:
            print(f"Error parsing data: {e}")
            break

    last_time = current_time

    if updated:
        # Update lines
        x_axis = range(len(roll_data))
        line_r.set_data(x_axis, roll_data)
        line_p.set_data(x_axis, pitch_data)
        line_y.set_data(x_axis, yaw_data)
        
        line_px.set_data(x_axis, px_data)
        line_py.set_data(x_axis, py_data)
        line_pz.set_data(x_axis, pz_data)

        # Dynamically scale position Y-axis based on current drift
        if len(px_data) > 0:
            min_p = min(min(px_data), min(py_data), min(pz_data))
            max_p = max(max(px_data), max(py_data), max(pz_data))
            margin = max(abs(max_p - min_p) * 0.1, 0.1)
            ax_pos.set_ylim(min_p - margin, max_p + margin)
    
    return line_r, line_p, line_y, line_px, line_py, line_pz

ani = animation.FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.show()