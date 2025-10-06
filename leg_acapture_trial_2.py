"""
leg_pose_capture.py

Real-time leg joint angle capture using MediaPipe + OpenCV.
Sends JSON packets via UDP to localhost:5005 and logs CSV.

Usage:
    pip install mediapipe opencv-python numpy
    python leg_capture.py --camera 0 --leg left

Features:
 - Detects pose landmarks via MediaPipe
 - Computes hip, knee, ankle angles (3D) for chosen leg
 - Smooths angles with exponential moving average
 - Overlays angles on video feed
 - Sends JSON via UDP to localhost:5005 (change host/port in args)
 - Logs CSV (output_angles.csv)

Notes:
 - MediaPipe provides normalized 3D coords; angles computed from 3D vectors.
 - Hip angle here is reported as angle between trunk (shoulder->hip) and thigh (hip->knee).
 - Knee angle is angle between thigh and shank (hip->knee vs knee->ankle).
 - Ankle angle is angle between shank and foot (knee->ankle vs ankle->foot_index).
 - Depending on your bionic leg kinematics, you may need to remap signs and reference frames.

Author: ChatGPT (assistant)
"""

import argparse
import csv
import json
import math
import socket
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np

# ----------------------------- Utilities ---------------------------------

def vec(a, b):
    """Return vector from a to b (3D)."""
    return np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]], dtype=float)


def angle_between(u, v):
    """Return angle in degrees between vectors u and v (3D)."""
    denom = (np.linalg.norm(u) * np.linalg.norm(v))
    if denom == 0:
        return 0.0
    cosv = np.dot(u, v) / denom
    cosv = np.clip(cosv, -1.0, 1.0)
    return math.degrees(math.acos(cosv))


# Exponential moving average smoother
class EMA:
    def __init__(self, alpha=0.5, init_val=0.0):
        self.alpha = alpha
        self.value = init_val
        self.initialized = False

    def update(self, x):
        if not self.initialized:
            self.value = x
            self.initialized = True
        else:
            self.value = self.alpha * x + (1 - self.alpha) * self.value
        return self.value


# ----------------------------- Main pipeline -----------------------------

def main(args):
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {args.camera}")
        return

    # UDP socket for sending angles
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_addr = (args.udp_host, args.udp_port)

    # CSV logging
    csv_file = open(args.csv, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['timestamp', 'hip_deg', 'knee_deg', 'ankle_deg'])

    # EMA smoothers
    hip_ema = EMA(alpha=args.smooth)
    knee_ema = EMA(alpha=args.smooth)
    ankle_ema = EMA(alpha=args.smooth)

    # For simple FPS calc
    prev_time = time.time()

    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            h, w, _ = image.shape

            # Default angles
            hip_angle = knee_angle = ankle_angle = 0.0
            detected = False

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                # Choose left or right landmarks
                if args.leg == 'left':
                    hip = lm[mp_pose.PoseLandmark.LEFT_HIP.value]
                    knee = lm[mp_pose.PoseLandmark.LEFT_KNEE.value]
                    ankle = lm[mp_pose.PoseLandmark.LEFT_ANKLE.value]
                    foot = lm[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value]
                else:
                    hip = lm[mp_pose.PoseLandmark.RIGHT_HIP.value]
                    knee = lm[mp_pose.PoseLandmark.RIGHT_KNEE.value]
                    ankle = lm[mp_pose.PoseLandmark.RIGHT_ANKLE.value]
                    foot = lm[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value]

                # also get shoulders for trunk vector
                left_sh = lm[mp_pose.PoseLandmark.LEFT_SHOULDER.value]
                right_sh = lm[mp_pose.PoseLandmark.RIGHT_SHOULDER.value]
                # trunk point as midpoint of shoulders
                trunk = np.array([ (left_sh.x + right_sh.x)/2, (left_sh.y + right_sh.y)/2, (left_sh.z + right_sh.z)/2 ])

                hip_p = np.array([hip.x, hip.y, hip.z])
                knee_p = np.array([knee.x, knee.y, knee.z])
                ankle_p = np.array([ankle.x, ankle.y, ankle.z])
                foot_p = np.array([foot.x, foot.y, foot.z])

                # Convert normalized coordinates to metric-like scale using image size for 2D scale
                # (Mediapipe's z is roughly proportional but not exact). Still, for angles we can use normalized 3D coords.

                # Vectors
                thigh = vec(hip_p, knee_p)       # hip -> knee
                shank = vec(knee_p, ankle_p)     # knee -> ankle
                foot_vec = vec(ankle_p, foot_p)   # ankle -> foot index
                trunk_vec = vec(trunk, hip_p)    # trunk -> hip (pointing downwards)

                # Angles
                hip_angle = angle_between(trunk_vec, thigh)
                knee_angle = angle_between(thigh, shank)
                ankle_angle = angle_between(shank, foot_vec)

                # Apply simple sign convention: flexion positive for knee (optional mapping downstream)
                detected = True

                # Draw landmarks and connections
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                # Annotate angles near joints
                def draw_text(p, text):
                    x = int(p[0] * w)
                    y = int(p[1] * h)
                    cv2.putText(image, text, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                draw_text(hip_p, f"Hip: {hip_angle:.1f}°")
                draw_text(knee_p, f"Knee: {knee_angle:.1f}°")
                draw_text(ankle_p, f"Ankle: {ankle_angle:.1f}°")

            # Smooth
            hip_s = hip_ema.update(hip_angle)
            knee_s = knee_ema.update(knee_angle)
            ankle_s = ankle_ema.update(ankle_angle)

            # Clip to realistic ranges (user adjust as per prosthesis)
            hip_s = float(np.clip(hip_s, args.hip_min, args.hip_max))
            knee_s = float(np.clip(knee_s, args.knee_min, args.knee_max))
            ankle_s = float(np.clip(ankle_s, args.ankle_min, args.ankle_max))

            # Send via UDP as JSON
            ts = time.time()
            payload = {
                'timestamp': ts,
                'detected': bool(detected),
                'leg': args.leg,
                'hip_deg': hip_s,
                'knee_deg': knee_s,
                'ankle_deg': ankle_s
            }
            try:
                sock.sendto(json.dumps(payload).encode('utf-8'), udp_addr)
            except Exception as e:
                print('UDP send error:', e)

            # Log CSV
            csv_writer.writerow([ts, hip_s, knee_s, ankle_s])

            # Overlay current values at top-left
            info = f"Hip: {hip_s:.1f}  Knee: {knee_s:.1f}  Ankle: {ankle_s:.1f}  Detected: {detected}"
            cv2.rectangle(image, (0,0), (520,40), (0,0,0), -1)
            cv2.putText(image, info, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # FPS
            now = time.time()
            fps = 1.0 / (now - prev_time) if now != prev_time else 0.0
            prev_time = now
            cv2.putText(image, f"FPS: {int(fps)}", (image.shape[1]-100,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            cv2.imshow('Leg Pose Capture', image)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

    csv_file.close()
    cap.release()
    cv2.destroyAllWindows()


# ----------------------------- CLI --------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Leg joint angle capture using MediaPipe.')
    parser.add_argument('--camera', type=int, default=0, help='Camera device index')
    parser.add_argument('--leg', choices=['left','right'], default='left', help='Which leg to track')
    parser.add_argument('--udp_host', type=str, default='127.0.0.1', help='UDP host to send JSON')
    parser.add_argument('--udp_port', type=int, default=5005, help='UDP port')
    parser.add_argument('--csv', type=str, default='output_angles.csv', help='CSV filename')
    parser.add_argument('--smooth', type=float, default=0.4, help='EMA smoothing alpha (0-1)')
    parser.add_argument('--hip_min', type=float, default=-90.0)
    parser.add_argument('--hip_max', type=float, default=180.0)
    parser.add_argument('--knee_min', type=float, default=0.0)
    parser.add_argument('--knee_max', type=float, default=160.0)
    parser.add_argument('--ankle_min', type=float, default=-60.0)
    parser.add_argument('--ankle_max', type=float, default=60.0)
    args = parser.parse_args()

    main(args)
