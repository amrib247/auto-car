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
px_data, py_data = [0.0], [0.0]

# Filter & Integration states
roll, pitch, yaw = 0.0, 0.0, 0.0
vx, vy = 0.0, 0.0
px, py = 0.0, 0.0
last_time = time.time()

# Calibration & ZUPT Variables
calibrating = True
calib_samples = 0
max_calib_samples = 200
acc_bias = [0.0, 0.0, 0.0]
gyro_bias = [0.0, 0.0, 0.0]

# ZUPT (Zero Velocity Update) Window
accel_mag_window = []
WINDOW_SIZE = 20
VARIANCE_THRESHOLD = 0.01 # Adjust this based on your car's vibration

print("KEEP SENSOR PERFECTLY STILL. Calibrating...")

# Plot Setup
fig, ax_pos = plt.subplots(figsize=(7, 7))
line_pos, = ax_pos.plot([], [], '-', lw=2, color='blue', label='Path')
scatter_head = ax_pos.scatter([], [], color='red', zorder=5, label='Current Pos')
txt_status = ax_pos.text(0.02, 0.95, '', transform=ax_pos.transAxes, fontsize=12)

ax_pos.set_title("2D Spatial Trajectory with ZUPT")
ax_pos.set_xlabel("X Position (meters)")
ax_pos.set_ylabel("Y Position (meters)")
ax_pos.grid(True)
ax_pos.legend(loc="upper right")
ax_pos.set_aspect('equal')

def update(frame):
    global roll, pitch, yaw, vx, vy, px, py, last_time
    global calibrating, calib_samples, acc_bias, gyro_bias
    
    current_time = time.time()
    dt = current_time - last_time
    updated = False
    is_stopped = False

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            decoded = data.decode('utf-8').strip()

            #print("Recieved!!")

            parts = decoded.split(',')
            
            if len(parts) >= 6:
                # Parse Data: Accel (mg -> g), Gyro (deg/s -> rad/s)
                ax = float(parts[0]) / 1000.0
                ay = float(parts[1]) / 1000.0
                az = float(parts[2]) / 1000.0
                gx = math.radians(float(parts[3]))
                gy = math.radians(float(parts[4]))
                gz = math.radians(float(parts[5]))

                if calibrating:
                    acc_bias[0] += ax; acc_bias[1] += ay; acc_bias[2] += (az - 1.0)
                    gyro_bias[0] += gx; gyro_bias[1] += gy; gyro_bias[2] += gz
                    
                    calib_samples += 1
                    if calib_samples >= max_calib_samples:
                        acc_bias = [b / max_calib_samples for b in acc_bias]
                        gyro_bias = [b / max_calib_samples for b in gyro_bias]
                        calibrating = False
                        print(f"Calibration Complete!\nAcc Bias: {acc_bias}\nGyro Bias: {gyro_bias}")
                    continue

                # Remove biases
                ax -= acc_bias[0]; ay -= acc_bias[1]; az -= acc_bias[2]
                gx -= gyro_bias[0]; gy -= gyro_bias[1]; gz -= gyro_bias[2]

                # 1. Mahony-style Orientation Update (Better than pure complementary)
                roll_acc = math.atan2(ay, az)
                pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))

                # Adaptive gain: trust gyro more during high acceleration
                acc_mag = math.sqrt(ax**2 + ay**2 + az**2)
                alpha = 0.99 if abs(acc_mag - 1.0) > 0.2 else 0.96

                roll = alpha * (roll + gx * dt) + (1 - alpha) * roll_acc
                pitch = alpha * (pitch + gy * dt) + (1 - alpha) * pitch_acc
                yaw = yaw + gz * dt

                # 2. Extract Linear Acceleration (Remove Gravity)
                lin_ax = (ax + math.sin(pitch)) * 9.81
                lin_ay = (ay - math.sin(roll) * math.cos(pitch)) * 9.81
                
                # 3. ZUPT: Dynamic Variance Check
                accel_mag_window.append(math.sqrt(lin_ax**2 + lin_ay**2))
                if len(accel_mag_window) > WINDOW_SIZE:
                    accel_mag_window.pop(0)
                
                variance = np.var(accel_mag_window) if len(accel_mag_window) == WINDOW_SIZE else 1.0

                if variance < VARIANCE_THRESHOLD:
                    is_stopped = True
                    vx = 0.0
                    vy = 0.0
                    lin_ax = 0.0
                    lin_ay = 0.0
                else:
                    is_stopped = False

                # 4. Integrate Velocity and Position
                vx += lin_ax * dt
                vy += lin_ay * dt

                # Leaky Integrator: If not actively accelerating hard, bleed velocity
                if abs(lin_ax) < 0.5: vx *= 0.75
                if abs(lin_ay) < 0.5: vy *= 0.75

                px += vx * dt
                py += vy * dt

                px_data.append(px)
                py_data.append(py)

                if len(px_data) > 500:
                    px_data.pop(0)
                    py_data.pop(0)
                
                updated = True
        except BlockingIOError:
            break 
        except Exception as e:
            pass

    last_time = current_time

    if updated and not calibrating:
        line_pos.set_data(px_data, py_data)
        scatter_head.set_offsets(np.c_[px_data[-1], py_data[-1]])
        
        status_text = "Status: STOPPED (ZUPT Active)" if is_stopped else f"Status: MOVING\nVel: {math.sqrt(vx**2 + vy**2):.2f} m/s"
        txt_status.set_text(status_text)
        txt_status.set_color('green' if is_stopped else 'red')

        if len(px_data) > 1:
            margin = 1.0 
            ax_pos.set_xlim(min(px_data) - margin, max(px_data) + margin)
            ax_pos.set_ylim(min(py_data) - margin, max(py_data) + margin)
    
    return line_pos, scatter_head, txt_status

ani = animation.FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.show()