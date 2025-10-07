import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ------------------- Configuration -------------------
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
BUFFER_SIZE = 200  # Number of points shown in plot
# ------------------------------------------------------

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Initialize data buffers for both legs
r_hip_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
r_knee_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
r_ankle_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)

l_hip_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
l_knee_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
l_ankle_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)

# Setup figure and two subplots
fig, (ax_r, ax_l) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle('Live Leg Joint Angles', fontsize=16)

# Right leg plot
line_r_hip, = ax_r.plot([], [], label='R-Hip')
line_r_knee, = ax_r.plot([], [], label='R-Knee')
line_r_ankle, = ax_r.plot([], [], label='R-Ankle')
ax_r.set_ylim(-100, 180)
ax_r.set_xlim(0, BUFFER_SIZE)
ax_r.set_ylabel('Angle (deg)')
ax_r.set_title('Right Leg')
ax_r.legend()
ax_r.grid(True)

# Left leg plot
line_l_hip, = ax_l.plot([], [], label='L-Hip')
line_l_knee, = ax_l.plot([], [], label='L-Knee')
line_l_ankle, = ax_l.plot([], [], label='L-Ankle')
ax_l.set_ylim(-100, 180)
ax_l.set_xlim(0, BUFFER_SIZE)
ax_l.set_xlabel('Frame')
ax_l.set_ylabel('Angle (deg)')
ax_l.set_title('Left Leg')
ax_l.legend()
ax_l.grid(True)

# List of all lines for blit
lines = [line_r_hip, line_r_knee, line_r_ankle, line_l_hip, line_l_knee, line_l_ankle]

# ------------------- Update function -------------------
def update(frame):
    try:
        data, _ = sock.recvfrom(4096)
        payload = json.loads(data.decode('utf-8'))

        # Update right leg
        r_hip_data.append(payload.get('r_hip', 0.0))
        r_knee_data.append(payload.get('r_knee', 0.0))
        r_ankle_data.append(payload.get('r_ankle', 0.0))

        # Update left leg
        l_hip_data.append(payload.get('l_hip', 0.0))
        l_knee_data.append(payload.get('l_knee', 0.0))
        l_ankle_data.append(payload.get('l_ankle', 0.0))

    except BlockingIOError:
        pass  # No data received this frame

    x = range(BUFFER_SIZE)

    # Update plot lines for right leg
    line_r_hip.set_data(x, list(r_hip_data))
    line_r_knee.set_data(x, list(r_knee_data))
    line_r_ankle.set_data(x, list(r_ankle_data))

    # Update plot lines for left leg
    line_l_hip.set_data(x, list(l_hip_data))
    line_l_knee.set_data(x, list(l_knee_data))
    line_l_ankle.set_data(x, list(l_ankle_data))

    return lines

# ------------------- Run animation -------------------
ani = animation.FuncAnimation(fig, update, interval=30, blit=True)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
