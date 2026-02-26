import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
from matplotlib.animation import FuncAnimation
import numpy as np

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/tty.usbmodem21403' 
BAUD_RATE = 115200
DT = 0.02  # Time step (matching interval=20ms)

# Global State for Integration
velocity = np.array([0.0, 0.0, 0.0])
position = np.array([0.0, 0.0, 0.0])

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"Error: {e}"); exit()

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

def update(frame):
    global velocity, position
    
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8').strip()
            accel = np.array([float(x) for x in line.split(',')]) # Assume m/s^2
            
            # 1. SIMPLE FILTER: Remove Gravity (Crude approximation)
            # In a real scenario, you'd use the Orientation matrix to subtract 9.8 from the Z-world-axis
            accel_linear = accel - np.array([0, 0, 9.8]) 
            
            # Deadzone to reduce drift when stationary
            accel_linear[np.abs(accel_linear) < 0.2] = 0 

            # 2. DOUBLE INTEGRATION
            velocity += accel_linear * DT
            position += velocity * DT
            
            # Friction/Damping (Helps stop the drift slightly)
            velocity *= 0.95 
            
        except: pass

    ax.clear()
    # Dynamic bounds based on position
    limit = max(max(np.abs(position)), 2)
    ax.set_xlim([-limit, limit]); ax.set_ylim([-limit, limit]); ax.set_zlim([-limit, limit])
    
    # Draw the "Sensor" at the calculated position
    # Vertices offset by the 'position' vector
    p = position
    box = np.array([[-0.2, -0.1, -0.05], [0.2, -0.1, -0.05], [0.2, 0.1, -0.05], [-0.2, 0.1, -0.05],
                    [-0.2, -0.1, 0.05], [0.2, -0.1, 0.05], [0.2, 0.1, 0.05], [-0.2, 0.1, 0.05]]) + p

    faces = [[box[j] for j in [0,1,2,3]], [box[j] for j in [4,5,6,7]], 
             [box[j] for j in [0,1,5,4]], [box[j] for j in [2,3,7,6]],
             [box[j] for j in [1,2,6,5]], [box[j] for j in [4,7,3,0]]]

    ax.add_collection3d(art3d.Poly3DCollection(faces, facecolors='magenta', edgecolors='black', alpha=0.8))
    ax.set_title(f"Position: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f}")

ani = FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.show()