import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/tty.usbmodem11403' 
BAUD_RATE = 115200
HISTORY_SIZE = 100
# Adjust this based on your MPU6050's Full Scale Range (Default is 16384 for +/- 2g)
LSB_PER_G = 16384.0
GRAVITY = 9.81

# Data buffers
accel_x = deque([0]*HISTORY_SIZE, maxlen=HISTORY_SIZE)
accel_y = deque([0]*HISTORY_SIZE, maxlen=HISTORY_SIZE)
accel_z = deque([0]*HISTORY_SIZE, maxlen=HISTORY_SIZE)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"Error: {e}")
    exit()

# Setup Figure
fig = plt.figure(figsize=(12, 6))
# Increase spacing between subplots
fig.subplots_adjust(wspace=0.3) 

ax_3d = fig.add_subplot(121, projection='3d')
ax_2d = fig.add_subplot(122)

def get_rotation_matrix(pitch, roll):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    return Ry @ Rx

def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            parts = [float(p) for p in line.split(',')]
            
            if len(parts) == 3:
                # --- CONVERSION TO m/s^2 ---
                # Convert raw bits to Gs, then to m/s^2
                ax_mps2 = [(p / LSB_PER_G) * GRAVITY for p in parts]
                
                accel_x.append(ax_mps2[0])
                accel_y.append(ax_mps2[1])
                accel_z.append(ax_mps2[2])

                # Clear axes for fresh redraw
                ax_3d.clear()
                ax_2d.clear()

                # --- 3D VISUALIZATION ---
                ax_3d.set_xlim([-2, 2]); ax_3d.set_ylim([-2, 2]); ax_3d.set_zlim([-2, 2])
                ax_3d.set_title("3D Orientation")

                # Calculate angles (uses same ratio regardless of units)
                pitch = np.arctan2(ax_mps2[1], np.sqrt(ax_mps2[0]**2 + ax_mps2[2]**2))
                roll = np.arctan2(-ax_mps2[0], ax_mps2[2])

                base_box = np.array([[-1, -0.5, -0.1], [1, -0.5, -0.1], [1, 0.5, -0.1], [-1, 0.5, -0.1],
                                    [-1, -0.5, 0.1], [1, -0.5, 0.1], [1, 0.5, 0.1], [-1, 0.5, 0.1]])

                rotated_box = base_box @ get_rotation_matrix(pitch, roll).T
                faces = [[rotated_box[j] for j in [0, 1, 2, 3]],
                         [rotated_box[j] for j in [4, 5, 6, 7]], 
                         [rotated_box[j] for j in [0, 1, 5, 4]],
                         [rotated_box[j] for j in [2, 3, 7, 6]],
                         [rotated_box[j] for j in [1, 2, 6, 5]],
                         [rotated_box[j] for j in [4, 7, 3, 0]]]

                poly = art3d.Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.5)
                ax_3d.add_collection3d(poly)

                # --- 2D ACCELERATION PLOT ---
                ax_2d.plot(list(accel_x), label='X', color='r')
                ax_2d.plot(list(accel_y), label='Y', color='g')
                ax_2d.plot(list(accel_z), label='Z', color='b')
                
                ax_2d.set_title("Acceleration ($m/s^2$)")
                ax_2d.set_xlabel("Samples")
                ax_2d.set_ylabel("Value ($m/s^2$)")
                ax_2d.legend(loc='upper right')
                
                # Auto-scaling with buffer
                all_data = list(accel_x) + list(accel_y) + list(accel_z)
                ax_2d.set_ylim(min(all_data) - 2, max(all_data) + 2)

        except Exception:
            pass

ani = FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()