import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/tty.usbmodem212203'
BAUD_RATE = 115200
HISTORY_SIZE = 100
LSB_PER_G = 8192.0  # 4G range configured in firmware (AFS_SEL=1)
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
fig.subplots_adjust(wspace=0.3)

ax_3d = fig.add_subplot(121, projection='3d')
ax_2d = fig.add_subplot(122)

# Pre-create 2D line objects so we can update them without clearing
line_x, = ax_2d.plot([], [], label='X', color='r')
line_y, = ax_2d.plot([], [], label='Y', color='g')
line_z, = ax_2d.plot([], [], label='Z', color='b')
ax_2d.set_title("Acceleration ($m/s^2$)")
ax_2d.set_xlabel("Samples")
ax_2d.set_ylabel("Value ($m/s^2$)")
ax_2d.set_xlim(0, HISTORY_SIZE)
ax_2d.legend(loc='upper right')

# Boot state
boot_messages = []
is_booting = True

def get_rotation_matrix(pitch, roll):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    return Ry @ Rx

def draw_boot_screen():
    ax_3d.clear()
    ax_2d.clear()

    # Left panel: title + spinner dots
    ax_3d.set_axis_off()
    dot_count = (len(boot_messages) % 4)
    ax_3d.text2D(0.5, 0.6, "MPU6050", transform=ax_3d.transAxes,
                 ha='center', va='center', fontsize=16, fontweight='bold', color='cyan')
    ax_3d.text2D(0.5, 0.45, "Initializing" + "." * dot_count,
                 transform=ax_3d.transAxes,
                 ha='center', va='center', fontsize=12, color='white')

    # Right panel: scrolling boot log
    ax_2d.set_axis_off()
    ax_2d.set_facecolor('#0d0d0d')
    fig.patch.set_facecolor('#1a1a2e')
    ax_3d.set_facecolor('#0d0d0d')

    ax_2d.text(0.05, 0.97, "Boot Log", transform=ax_2d.transAxes,
               ha='left', va='top', fontsize=10, color='gray', style='italic')

    visible = boot_messages[-14:]  # Show last 14 messages
    for i, msg in enumerate(visible):
        y = 0.88 - i * 0.063
        is_error = any(kw in msg.lower() for kw in ['fail', 'snoozing'])
        is_calib = 'calib' in msg.lower()
        color = '#ff5555' if is_error else ('#ffb86c' if is_calib else '#50fa7b')
        ax_2d.text(0.05, y, f"> {msg}", transform=ax_2d.transAxes,
                   ha='left', va='top', fontsize=9, color=color,
                   fontfamily='monospace')

def update(frame):
    global is_booting

    # Drain the entire serial buffer each frame — only render the latest sample.
    # This prevents buffer buildup and latency when the MPU sends faster than we render.
    latest_values = None
    try:
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    latest_values = [float(p) for p in parts]
                except ValueError:
                    pass
            elif line:
                boot_messages.append(line)
                print(f"[STM32] {line}")
    except Exception:
        pass

    if latest_values is not None:
        if is_booting:
            is_booting = False
            fig.patch.set_facecolor('white')

        ax_mps2 = [(p / LSB_PER_G) * GRAVITY for p in latest_values]
        accel_x.append(ax_mps2[0])
        accel_y.append(ax_mps2[1])
        accel_z.append(ax_mps2[2])

        # --- 2D: update existing line objects instead of clearing ---
        xs = list(range(len(accel_x)))
        line_x.set_data(xs, list(accel_x))
        line_y.set_data(xs, list(accel_y))
        line_z.set_data(xs, list(accel_z))
        all_data = list(accel_x) + list(accel_y) + list(accel_z)
        ax_2d.set_ylim(min(all_data) - 2, max(all_data) + 2)

        # --- 3D: must clear to redraw the box ---
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
        ax_3d.clear()
        ax_3d.set_xlim([-2, 2]); ax_3d.set_ylim([-2, 2]); ax_3d.set_zlim([-2, 2])
        ax_3d.set_title("3D Orientation")
        poly = art3d.Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.5)
        ax_3d.add_collection3d(poly)

    elif is_booting:
        draw_boot_screen()

ani = FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()
