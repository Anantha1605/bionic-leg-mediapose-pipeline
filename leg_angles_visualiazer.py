import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Buffer size for plotting (number of points shown)
BUFFER_SIZE = 200

# Setup UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Data buffers
hip_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
knee_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
ankle_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)

fig, ax = plt.subplots()
line_hip, = ax.plot([], [], label='Hip')
line_knee, = ax.plot([], [], label='Knee')
line_ankle, = ax.plot([], [], label='Ankle')
ax.set_ylim(-100, 180)
ax.set_xlim(0, BUFFER_SIZE)
ax.set_xlabel('Frame')
ax.set_ylabel('Angle (deg)')
ax.legend()
ax.set_title('Live Leg Joint Angles')

def update(frame):
    # Try to receive UDP packet
    try:
        data, _ = sock.recvfrom(4096)
        payload = json.loads(data.decode('utf-8'))
        hip_data.append(payload['hip_deg'])
        knee_data.append(payload['knee_deg'])
        ankle_data.append(payload['ankle_deg'])
    except BlockingIOError:
        pass  # No data received this frame

    # Update plot data
    x = range(len(hip_data))
    line_hip.set_data(x, list(hip_data))
    line_knee.set_data(x, list(knee_data))
    line_ankle.set_data(x, list(ankle_data))
    return line_hip, line_knee, line_ankle

ani = animation.FuncAnimation(fig, update, interval=30, blit=True)
plt.show()