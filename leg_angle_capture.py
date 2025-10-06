import cv2
import mediapipe as mp
import numpy as np
import csv
from datetime import datetime

class LegJointAngleDetector:
    def __init__(self):
        """
        Initialize MediaPipe Pose model for human pose detection
        No external model files required - MediaPipe handles everything
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
        
        # Initialize CSV file
        self.csv_filename = "output_angles.csv"
        self.initialize_csv()
        
        # Store previous angles for smoothing
        self.prev_angles = None
        self.frame_count = 0
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'frame', 
                           'right_hip_angle', 'right_knee_angle', 'right_ankle_angle',
                           'left_hip_angle', 'left_knee_angle', 'left_ankle_angle'])
        print(f"CSV file '{self.csv_filename}' initialized")
    
    def calculate_angle(self, p1, p2, p3):
        """
        Calculate angle at point p2 formed by points p1-p2-p3
        Returns angle in degrees
        """
        if p1 is None or p2 is None or p3 is None:
            return None
        
        # Convert points to numpy arrays
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)
        
        # Calculate vectors
        v1 = p1 - p2
        v2 = p3 - p2
        
        # Calculate angle using dot product
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        return np.degrees(angle)
    
    def detect_pose(self, frame):
        """Detect human pose using MediaPipe"""
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        results = self.pose.process(image_rgb)
        image_rgb.flags.writeable = True
        
        if not results.pose_landmarks:
            return None
        
        return results.pose_landmarks.landmark
    
    def landmark_to_pixel(self, landmark, frame_width, frame_height):
        """Convert normalized landmark to pixel coordinates"""
        if landmark.visibility < 0.5:
            return None
        return (int(landmark.x * frame_width), int(landmark.y * frame_height))
    
    def calculate_leg_angles(self, landmarks, frame_width, frame_height):
        """Calculate hip, knee, and ankle angles for both legs"""
        if landmarks is None:
            return {
                'right_hip': None,
                'right_knee': None,
                'right_ankle': None,
                'left_hip': None,
                'left_knee': None,
                'left_ankle': None
            }
        
        angles = {}
        
        # Convert landmarks to pixel coordinates
        right_shoulder = self.landmark_to_pixel(landmarks[self.BODY_PARTS["RightShoulder"]], frame_width, frame_height)
        right_hip = self.landmark_to_pixel(landmarks[self.BODY_PARTS["RHip"]], frame_width, frame_height)
        right_knee = self.landmark_to_pixel(landmarks[self.BODY_PARTS["RKnee"]], frame_width, frame_height)
        right_ankle = self.landmark_to_pixel(landmarks[self.BODY_PARTS["RAnkle"]], frame_width, frame_height)
        
        left_shoulder = self.landmark_to_pixel(landmarks[self.BODY_PARTS["LeftShoulder"]], frame_width, frame_height)
        left_hip = self.landmark_to_pixel(landmarks[self.BODY_PARTS["LHip"]], frame_width, frame_height)
        left_knee = self.landmark_to_pixel(landmarks[self.BODY_PARTS["LKnee"]], frame_width, frame_height)
        left_ankle = self.landmark_to_pixel(landmarks[self.BODY_PARTS["LAnkle"]], frame_width, frame_height)
        
        # Right leg angles
        # Hip angle: shoulder - hip - knee
        if right_shoulder and right_hip and right_knee:
            angles['right_hip'] = self.calculate_angle(right_shoulder, right_hip, right_knee)
        else:
            angles['right_hip'] = None
        
        # Knee angle: hip - knee - ankle
        if right_hip and right_knee and right_ankle:
            angles['right_knee'] = self.calculate_angle(right_hip, right_knee, right_ankle)
        else:
            angles['right_knee'] = None
        
        # Ankle angle: knee - ankle - foot (using vertical reference)
        if right_knee and right_ankle:
            foot_ref = (right_ankle[0], right_ankle[1] + 50)
            angles['right_ankle'] = self.calculate_angle(right_knee, right_ankle, foot_ref)
        else:
            angles['right_ankle'] = None
        
        # Left leg angles
        # Hip angle: shoulder - hip - knee
        if left_shoulder and left_hip and left_knee:
            angles['left_hip'] = self.calculate_angle(left_shoulder, left_hip, left_knee)
        else:
            angles['left_hip'] = None
        
        # Knee angle: hip - knee - ankle
        if left_hip and left_knee and left_ankle:
            angles['left_knee'] = self.calculate_angle(left_hip, left_knee, left_ankle)
        else:
            angles['left_knee'] = None
        
        # Ankle angle: knee - ankle - foot (using vertical reference)
        if left_knee and left_ankle:
            foot_ref = (left_ankle[0], left_ankle[1] + 50)
            angles['left_ankle'] = self.calculate_angle(left_knee, left_ankle, foot_ref)
        else:
            angles['left_ankle'] = None
        
        return angles
    
    def save_angles_to_csv(self, angles):
        """Save joint angles to CSV file"""
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
    
    def draw_skeleton(self, frame, landmarks, angles):
        """Draw skeleton and labels on frame"""
        if landmarks is None:
            return frame
        
        frame_height, frame_width = frame.shape[:2]
        
        # Get joint positions
        joint_positions = {
            'right_hip': self.landmark_to_pixel(landmarks[self.BODY_PARTS["RHip"]], frame_width, frame_height),
            'right_knee': self.landmark_to_pixel(landmarks[self.BODY_PARTS["RKnee"]], frame_width, frame_height),
            'right_ankle': self.landmark_to_pixel(landmarks[self.BODY_PARTS["RAnkle"]], frame_width, frame_height),
            'left_hip': self.landmark_to_pixel(landmarks[self.BODY_PARTS["LHip"]], frame_width, frame_height),
            'left_knee': self.landmark_to_pixel(landmarks[self.BODY_PARTS["LKnee"]], frame_width, frame_height),
            'left_ankle': self.landmark_to_pixel(landmarks[self.BODY_PARTS["LAnkle"]], frame_width, frame_height)
        }
        
        # Draw leg connections
        leg_connections = [
            ('right_hip', 'right_knee'),
            ('right_knee', 'right_ankle'),
            ('left_hip', 'left_knee'),
            ('left_knee', 'left_ankle')
        ]
        
        for connection in leg_connections:
            p1 = joint_positions[connection[0]]
            p2 = joint_positions[connection[1]]
            if p1 and p2:
                cv2.line(frame, p1, p2, (0, 255, 0), 3)
        
        # Draw and label joints
        joint_labels = {
            'right_hip': ("R-Hip", angles.get('right_hip')),
            'right_knee': ("R-Knee", angles.get('right_knee')),
            'right_ankle': ("R-Ankle", angles.get('right_ankle')),
            'left_hip': ("L-Hip", angles.get('left_hip')),
            'left_knee': ("L-Knee", angles.get('left_knee')),
            'left_ankle': ("L-Ankle", angles.get('left_ankle'))
        }
        
        for joint_name, (label, angle) in joint_labels.items():
            pos = joint_positions[joint_name]
            if pos:
                cv2.circle(frame, pos, 8, (0, 0, 255), -1)
                cv2.circle(frame, pos, 10, (255, 255, 255), 2)
                
                text = f"{label}"
                if angle is not None:
                    text += f": {angle:.1f}°"
                
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                text_x = pos[0] + 15
                text_y = pos[1]
                
                cv2.rectangle(frame, 
                            (text_x - 2, text_y - text_size[1] - 2),
                            (text_x + text_size[0] + 2, text_y + 2),
                            (0, 0, 0), -1)
                
                cv2.putText(frame, text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return frame
    
    def run(self, video_source=0):
        """
        Run the leg joint angle detector
        video_source: 0 for webcam, or path to video file
        """
        cap = cv2.VideoCapture(video_source)
        
        if not cap.isOpened():
            print("Error: Could not open video source")
            return
        
        print("Starting leg joint angle detection with MediaPipe...")
        print("Press 'q' to quit")
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("End of video or cannot read frame")
                    break
                
                self.frame_count += 1
                frame_height, frame_width = frame.shape[:2]
                
                landmarks = self.detect_pose(frame)
                
                angles = self.calculate_leg_angles(landmarks, frame_width, frame_height)
                
                self.save_angles_to_csv(angles)
                
                frame = self.draw_skeleton(frame, landmarks, angles)
                
                cv2.rectangle(frame, (5, 5), (250, 40), (0, 0, 0), -1)
                cv2.putText(frame, f"Frame: {self.frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("Leg Joint Angle Detection - MediaPipe", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.pose.close()
            print(f"\nDetection completed. Angles saved to '{self.csv_filename}'")
            print(f"Total frames processed: {self.frame_count}")


if __name__ == "__main__":
    detector = LegJointAngleDetector()
    detector.run(video_source=0)