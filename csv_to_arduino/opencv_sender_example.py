# opencv_sender_example.py
# Example: send angles per-frame over UDP to relay (low latency).
# integrate the send_angle_message(...) call in your opencv frame loop.

import socket, time, json

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_angle_message(frame_idx,angles):
    """
    angles: list or tuple of 6 floats in order:
      right_hip,right_knee,right_ankle,left_hip,left_knee,left_ankle
    """
    # choose format: compact CSV is tiny and easy for relay parser
    # timestamp optional; we include frame idx + 6 angles
    payload = ",".join([str(frame_idx)] + [f"{a:.4f}" for a in angles])
    sock.sendto(payload.encode('utf-8'), (UDP_IP, UDP_PORT))

# ---------- quick demo sending fake data at 30 FPS -------------
if __name__ == "__main__":
    try:
        frame = 0
        while True:
            # replace next line by your real frame angles
            fake_angles = (173.7 + 1.0*(frame%5),166.6,170.6,170.3,168.9,170.6)
            send_angle_message(frame, fake_angles)
            frame += 1
            time.sleep(1.0/30.0)  # simulate 30 fps
    except KeyboardInterrupt:
        print("sender stopped")
