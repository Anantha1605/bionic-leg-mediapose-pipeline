import cv2
import mediapipe as mp
import numpy as np
import csv
from datetime import datetime
import socket
import json

class LegJointAngleDetector:
    def __init__(self, udp_ip="127.0.0.1", udp_port=5005, alpha=0.2):
        """
        Initialize MediaPipe Pose model and UDP socket
        alpha: smoothing factor for exponential moving average (0 < alpha <= 1)
        """
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # MediaPipe Pose landmark indices
        self.BODY_PARTS = {
            "RHip": 24,
            "RKnee": 26,
            "RAnkle": 28,
            "LHip": 23,
            "LKnee": 25,
            "LAnkle": 27,
            "RightShoulder": 12,
            "LeftShoulder": 11,
            "Nose": 0
        }
        
        # CSV for logging
        self.csv_filename = "output_angles.csv"
        self.initialize_csv()
        
        # For smoothing
        self.prev_angles = None
        self.alpha = alpha
        self.frame_count = 0
        
        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'frame', 
                           'right_hip_angle', 'right_knee_angle', 'right_ankle_angle',
                           'left_hip_angle', 'left_knee_angle', 'left_ankle_angle'])
        print(f"CSV file '{self.csv_filename}' initialized")
    
    def calculate_angle(self, p1, p2, p3):
        if p1 is None or p2 is None or p3 is None:
            return None
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)
        v1 = p1 - p2
        v2 = p3 - p2
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return np.degrees(angle)
    
    def detect_pose(self, frame):
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.pose.process(image_rgb)
        image_rgb.flags.writeable = True
        if not results.pose_landmarks:
            return None
        return results.pose_landmarks.landmark
    
    def landmark_to_pixel(self, landmark, frame_width, frame_height):
        if landmark.visibility < 0.5:
            return None
        return (int(landmark.x * frame_width), int(landmark.y * frame_height))
    
    def calculate_leg_angles(self, landmarks, frame_width, frame_height):
        if landmarks is None:
            return {k: None for k in ['right_hip','right_knee','right_ankle','left_hip','left_knee','left_ankle']}
        
        # Helper to get pixel
        def get_point(name):
            return self.landmark_to_pixel(landmarks[self.BODY_PARTS[name]], frame_width, frame_height)
        
        angles = {}
        right_shoulder = get_point("RightShoulder")
        right_hip = get_point("RHip")
        right_knee = get_point("RKnee")
        right_ankle = get_point("RAnkle")
        
        left_shoulder = get_point("LeftShoulder")
        left_hip = get_point("LHip")
        left_knee = get_point("LKnee")
        left_ankle = get_point("LAnkle")
        
        # Right leg
        angles['right_hip'] = self.calculate_angle(right_shoulder, right_hip, right_knee) if right_shoulder and right_hip and right_knee else None
        angles['right_knee'] = self.calculate_angle(right_hip, right_knee, right_ankle) if right_hip and right_knee and right_ankle else None
        angles['right_ankle'] = self.calculate_angle(right_knee, right_ankle, (right_ankle[0], right_ankle[1]+50)) if right_knee and right_ankle else None
        
        # Left leg
        angles['left_hip'] = self.calculate_angle(left_shoulder, left_hip, left_knee) if left_shoulder and left_hip and left_knee else None
        angles['left_knee'] = self.calculate_angle(left_hip, left_knee, left_ankle) if left_hip and left_knee and left_ankle else None
        angles['left_ankle'] = self.calculate_angle(left_knee, left_ankle, (left_ankle[0], left_ankle[1]+50)) if left_knee and left_ankle else None
        
        # Smooth angles with EMA
        if self.prev_angles is None:
            self.prev_angles = angles
        else:
            for k in angles:
                if angles[k] is not None:
                    prev = self.prev_angles[k] if self.prev_angles[k] is not None else angles[k]
                    angles[k] = self.alpha*angles[k] + (1-self.alpha)*prev
            self.prev_angles = angles
        
        return angles
    
    def save_angles_to_csv(self, angles):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        with open(self.csv_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                self.frame_count,
                angles.get('right_hip'),
                angles.get('right_knee'),
                angles.get('right_ankle'),
                angles.get('left_hip'),
                angles.get('left_knee'),
                angles.get('left_ankle')
            ])
    
    def send_angles_udp(self, angles):
        """Send angles over UDP (right leg for now)"""
        data = {
            'hip_deg': angles.get('right_hip', 0.0),
            'knee_deg': angles.get('right_knee', 0.0),
            'ankle_deg': angles.get('right_ankle', 0.0)
        }
        try:
            self.sock.sendto(json.dumps(data).encode('utf-8'), (self.udp_ip, self.udp_port))
        except Exception as e:
            print("UDP send error:", e)
    
    def draw_skeleton(self, frame, landmarks, angles):
        if landmarks is None:
            return frame
        frame_height, frame_width = frame.shape[:2]
        def get_point(name):
            return self.landmark_to_pixel(landmarks[self.BODY_PARTS[name]], frame_width, frame_height)
        joint_positions = {
            'right_hip': get_point("RHip"),
            'right_knee': get_point("RKnee"),
            'right_ankle': get_point("RAnkle"),
            'left_hip': get_point("LHip"),
            'left_knee': get_point("LKnee"),
            'left_ankle': get_point("LAnkle")
        }
        for p1_name, p2_name in [('right_hip','right_knee'),('right_knee','right_ankle'),('left_hip','left_knee'),('left_knee','left_ankle')]:
            p1, p2 = joint_positions[p1_name], joint_positions[p2_name]
            if p1 and p2:
                cv2.line(frame, p1, p2, (0,255,0),3)
        for joint_name, angle in angles.items():
            pos = joint_positions[joint_name]
            if pos:
                cv2.circle(frame, pos, 8, (0,0,255), -1)
                cv2.circle(frame, pos, 10, (255,255,255),2)
                text = f"{joint_name}:{angle:.1f}°" if angle is not None else joint_name
                cv2.putText(frame, text, (pos[0]+15,pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),2)
        return frame
    
    def run(self, video_source=0):
        cap = cv2.VideoCapture(video_source)
        if not cap.isOpened():
            print("Error: Could not open video source")
            return
        print("Press 'q' to quit")
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                self.frame_count += 1
                frame_height, frame_width = frame.shape[:2]
                landmarks = self.detect_pose(frame)
                angles = self.calculate_leg_angles(landmarks, frame_width, frame_height)
                self.save_angles_to_csv(angles)
                self.send_angles_udp(angles)
                frame = self.draw_skeleton(frame, landmarks, angles)
                cv2.putText(frame, f"Frame:{self.frame_count}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
                cv2.imshow("Leg Joint Angle Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.pose.close()
            print("Detection completed.")

if __name__ == "__main__":
    detector = LegJointAngleDetector()
    detector.run(video_source=0)
