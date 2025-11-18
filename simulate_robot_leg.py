#!/usr/bin/env python3
"""
simulate_robot_leg.py

Real-time visual simulation of bionic leg movement using CSV joint angle data.
Displays side-view of both legs with 3 joints each (hip, knee, ankle).

Controls:
    SPACE     - Pause/Resume playback
    LEFT      - Step backward one frame
    RIGHT     - Step forward one frame
    R         - Reset to beginning
    Q/ESC     - Quit
    UP/DOWN   - Increase/decrease playback speed
    G         - Toggle angle graph overlay

Usage:
    python simulate_robot_leg.py
    python simulate_robot_leg.py --csv output_angles.csv --fps 30
    python simulate_robot_leg.py --csv output_angles.csv --3d

Author: Generated for bionic-leg-mediapose-pipeline
Date: 2025-11-18
"""

import pygame
import csv
import math
import argparse
import sys
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from collections import deque

# ============= CONFIGURATION =============q

# Window settings
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60  # Display FPS (independent of playback speed)

# Robot dimensions (in pixels for visualization)
SEGMENT_LENGTHS = {
    'hip_to_knee': 60,   # Thigh length
    'knee_to_ankle': 70, # Shin length
    'ankle_to_foot': 40  # Foot length
}

# Visual settings
COLORS = {
    'background': (20, 20, 30),
    'torso': (100, 100, 120),
    'right_leg': (220, 80, 80),       # Red for right
    'right_leg_dark': (150, 50, 50),  # Dark red for shadows
    'right_leg_light': (255, 140, 140), # Light red for highlights
    'left_leg': (80, 80, 220),        # Blue for left
    'left_leg_dark': (50, 50, 150),   # Dark blue for shadows
    'left_leg_light': (140, 140, 255), # Light blue for highlights
    'joint': (60, 60, 70),            # Dark gray joints (servo motors)
    'joint_light': (100, 100, 110),   # Light gray for servo highlights
    'metal': (180, 180, 190),         # Metallic color
    'bolt': (40, 40, 45),             # Bolt/screw color
    'ground': (80, 80, 80),
    'text': (255, 255, 255),
    'graph_bg': (40, 40, 50, 180),
    'grid': (60, 60, 70)
}

JOINT_RADIUS = 18        # Larger for servo motor appearance
BONE_WIDTH = 16          # Much thicker segments
SCALE_FACTOR = 2.5       # Scale up robot for visibility

# CSV column names
CSV_COLUMNS = [
    'right_hip_angle',
    'right_knee_angle',
    'right_ankle_angle',
    'left_hip_angle',
    'left_knee_angle',
    'left_ankle_angle'
]

# ============= DATA STRUCTURES =============

@dataclass
class JointAngles:
    """Joint angles for one frame"""
    frame: int
    timestamp: str
    right_hip: float
    right_knee: float
    right_ankle: float
    left_hip: float
    left_knee: float
    left_ankle: float

@dataclass
class Point2D:
    """2D point"""
    x: float
    y: float
    
    def to_tuple(self) -> Tuple[int, int]:
        return (int(self.x), int(self.y))

@dataclass
class LegSegments:
    """Calculated positions for one leg"""
    hip: Point2D
    knee: Point2D
    ankle: Point2D
    foot: Point2D

# ============= CSV LOADER =============

def load_csv_data(csv_path: str) -> List[JointAngles]:
    """Load joint angles from CSV file"""
    frames = []
    
    try:
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            
            for row in reader:
                try:
                    frame = JointAngles(
                        frame=int(row.get('frame', len(frames) + 1)),
                        timestamp=row.get('timestamp', ''),
                        right_hip=float(row.get('right_hip_angle', 90)),
                        right_knee=float(row.get('right_knee_angle', 90)),
                        right_ankle=float(row.get('right_ankle_angle', 90)),
                        left_hip=float(row.get('left_hip_angle', 90)),
                        left_knee=float(row.get('left_knee_angle', 90)),
                        left_ankle=float(row.get('left_ankle_angle', 90))
                    )
                    frames.append(frame)
                except (ValueError, KeyError) as e:
                    print(f"Warning: Skipping malformed row: {e}")
                    continue
        
        print(f"Loaded {len(frames)} frames from {csv_path}")
        return frames
    
    except FileNotFoundError:
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        sys.exit(1)

# ============= FORWARD KINEMATICS =============

def calculate_leg_positions(hip_pos: Point2D, hip_angle: float, knee_angle: float, 
                            ankle_angle: float) -> LegSegments:
    """
    Calculate leg segment positions using forward kinematics.
    
    Angle conventions:
    - hip_angle: 0° = straight down, positive = forward
    - knee_angle: 180° = straight, less = bent
    - ankle_angle: 90° = perpendicular, varies for foot orientation
    """
    
    # Convert angles to radians and adjust for coordinate system
    # Hip: 0° should point downward in screen coords
    hip_rad = math.radians(180 - hip_angle)
    
    # Thigh segment (hip to knee)
    thigh_len = SEGMENT_LENGTHS['hip_to_knee'] * SCALE_FACTOR
    knee_x = hip_pos.x + thigh_len * math.sin(hip_rad)
    knee_y = hip_pos.y + thigh_len * math.cos(hip_rad)
    knee_pos = Point2D(knee_x, knee_y)
    
    # Shin segment (knee to ankle)
    # Knee angle affects the shin direction relative to thigh
    shin_len = SEGMENT_LENGTHS['knee_to_ankle'] * SCALE_FACTOR
    knee_bend = math.radians(180 - knee_angle)  # Amount of knee bend
    shin_angle = hip_rad + knee_bend
    
    ankle_x = knee_x + shin_len * math.sin(shin_angle)
    ankle_y = knee_y + shin_len * math.cos(shin_angle)
    ankle_pos = Point2D(ankle_x, ankle_y)
    
    # Foot segment (ankle to foot tip)
    foot_len = SEGMENT_LENGTHS['ankle_to_foot'] * SCALE_FACTOR
    ankle_bend = math.radians(ankle_angle - 90)  # 90° = perpendicular
    foot_angle = shin_angle + ankle_bend
    
    foot_x = ankle_x + foot_len * math.cos(foot_angle)
    foot_y = ankle_y + foot_len * math.sin(foot_angle)
    foot_pos = Point2D(foot_x, foot_y)
    
    return LegSegments(hip_pos, knee_pos, ankle_pos, foot_pos)

# ============= RENDERING =============

class RobotSimulator:
    """Main simulator class"""
    
    def __init__(self, csv_path: str, playback_fps: float = 30):
        pygame.init()
        
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Bionic Leg Simulator - MediaPose Data")
        self.clock = pygame.time.Clock()
        
        # Load data
        self.frames = load_csv_data(csv_path)
        if not self.frames:
            print("Error: No frames loaded")
            sys.exit(1)
        
        # Playback state
        self.current_frame = 0
        self.playing = True
        self.playback_fps = playback_fps
        self.frame_timer = 0.0
        self.speed_multiplier = 1.0
        
        # UI state
        self.show_graph = False
        self.angle_history = {
            'right_hip': deque(maxlen=100),
            'right_knee': deque(maxlen=100),
            'right_ankle': deque(maxlen=100),
            'left_hip': deque(maxlen=100),
            'left_knee': deque(maxlen=100),
            'left_ankle': deque(maxlen=100)
        }
        
        # Font
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Torso position (center of screen, upper area)
        self.torso_x = WINDOW_WIDTH // 2
        self.torso_y = 200
    
    def get_current_angles(self) -> JointAngles:
        """Get current frame's joint angles"""
        return self.frames[self.current_frame]
    
    def draw_ground(self):
        """Draw ground reference line with platform"""
        ground_y = WINDOW_HEIGHT - 100
        
        # Ground platform with 3D effect
        platform_height = 20
        platform_points = [
            (0, ground_y),
            (WINDOW_WIDTH, ground_y),
            (WINDOW_WIDTH, ground_y + platform_height),
            (0, ground_y + platform_height)
        ]
        pygame.draw.polygon(self.screen, (70, 70, 80), platform_points)
        
        # Platform top edge (lighter)
        pygame.draw.line(self.screen, (100, 100, 110), 
                        (0, ground_y), (WINDOW_WIDTH, ground_y), 3)
        
        # Platform grid pattern
        for x in range(0, WINDOW_WIDTH, 80):
            pygame.draw.line(self.screen, (60, 60, 70), 
                           (x, ground_y), (x, ground_y + platform_height), 1)
        
        # Grid lines for reference (vertical)
        for x in range(0, WINDOW_WIDTH, 50):
            pygame.draw.line(self.screen, COLORS['grid'], 
                           (x, ground_y - 300), (x, ground_y), 1)
        
        # Horizontal reference lines
        for y in range(int(ground_y - 300), int(ground_y), 50):
            pygame.draw.line(self.screen, (50, 50, 60),
                           (0, y), (WINDOW_WIDTH, y), 1)
    
    def draw_torso(self):
        """Draw robot torso with 3D appearance"""
        torso_height = 80
        torso_width = 60
        torso_x = self.torso_x - torso_width // 2
        torso_y = self.torso_y - torso_height
        
        # Main torso body
        torso_rect = pygame.Rect(torso_x, torso_y, torso_width, torso_height)
        
        # Draw torso with gradient effect
        pygame.draw.rect(self.screen, (80, 80, 90), torso_rect, border_radius=5)
        
        # Light highlight on left side
        highlight_rect = pygame.Rect(torso_x, torso_y, torso_width // 3, torso_height)
        pygame.draw.rect(self.screen, (120, 120, 140), highlight_rect, border_radius=5)
        
        # Dark shadow on right side
        shadow_rect = pygame.Rect(torso_x + 2 * torso_width // 3, torso_y, 
                                  torso_width // 3, torso_height)
        pygame.draw.rect(self.screen, (60, 60, 70), shadow_rect, border_radius=5)
        
        # Torso outline
        pygame.draw.rect(self.screen, (0, 0, 0), torso_rect, 3, border_radius=5)
        
        # Control panel details
        panel_rect = pygame.Rect(torso_x + 10, torso_y + 15, torso_width - 20, 25)
        pygame.draw.rect(self.screen, (40, 40, 50), panel_rect, border_radius=2)
        pygame.draw.rect(self.screen, (0, 0, 0), panel_rect, 1, border_radius=2)
        
        # LED indicators
        led_colors = [(0, 255, 0), (255, 200, 0), (0, 150, 255)]
        for i, led_color in enumerate(led_colors):
            led_x = torso_x + 15 + i * 15
            led_y = torso_y + 25
            pygame.draw.circle(self.screen, led_color, (led_x, led_y), 3)
            pygame.draw.circle(self.screen, (0, 0, 0), (led_x, led_y), 3, 1)
        
        # Ventilation slots
        for i in range(4):
            slot_y = torso_y + 50 + i * 6
            pygame.draw.line(self.screen, (30, 30, 40),
                           (torso_x + 10, slot_y),
                           (torso_x + torso_width - 10, slot_y), 2)
        
        # Hip mounting brackets
        right_hip_x = self.torso_x + 20
        left_hip_x = self.torso_x - 20
        hip_y = self.torso_y
        
        # Draw hip brackets
        for hip_x in [right_hip_x, left_hip_x]:
            bracket_rect = pygame.Rect(hip_x - 8, hip_y - 8, 16, 16)
            pygame.draw.rect(self.screen, COLORS['metal'], bracket_rect, border_radius=2)
            pygame.draw.rect(self.screen, (0, 0, 0), bracket_rect, 2, border_radius=2)
            pygame.draw.circle(self.screen, COLORS['bolt'], (hip_x, hip_y), 4)
            pygame.draw.circle(self.screen, (0, 0, 0), (hip_x, hip_y), 4, 1)
        
        return Point2D(right_hip_x, hip_y), Point2D(left_hip_x, hip_y)
    
    def draw_thick_segment(self, p1: Point2D, p2: Point2D, width: int, 
                          base_color: Tuple[int, int, int], dark_color: Tuple[int, int, int],
                          light_color: Tuple[int, int, int]):
        """Draw a thick 3D-looking segment with shading"""
        # Calculate perpendicular offset for thickness
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        length = math.sqrt(dx*dx + dy*dy)
        if length == 0:
            return
        
        # Perpendicular unit vector
        perp_x = -dy / length
        perp_y = dx / length
        
        # Create polygon for the segment with rounded ends
        half_width = width / 2
        
        # Main segment polygon (rectangle)
        points = [
            (p1.x + perp_x * half_width, p1.y + perp_y * half_width),
            (p2.x + perp_x * half_width, p2.y + perp_y * half_width),
            (p2.x - perp_x * half_width, p2.y - perp_y * half_width),
            (p1.x - perp_x * half_width, p1.y - perp_y * half_width)
        ]
        
        # Draw shadow/dark side
        shadow_points = [
            (p1.x - perp_x * half_width, p1.y - perp_y * half_width),
            (p2.x - perp_x * half_width, p2.y - perp_y * half_width),
            (p2.x, p2.y),
            (p1.x, p1.y)
        ]
        pygame.draw.polygon(self.screen, dark_color, shadow_points)
        
        # Draw main body
        pygame.draw.polygon(self.screen, base_color, points)
        
        # Draw highlight/light side
        highlight_points = [
            (p1.x + perp_x * half_width, p1.y + perp_y * half_width),
            (p2.x + perp_x * half_width, p2.y + perp_y * half_width),
            (p2.x + perp_x * (half_width * 0.5), p2.y + perp_y * (half_width * 0.5)),
            (p1.x + perp_x * (half_width * 0.5), p1.y + perp_y * (half_width * 0.5))
        ]
        pygame.draw.polygon(self.screen, light_color, highlight_points)
        
        # Draw rounded ends
        pygame.draw.circle(self.screen, base_color, p1.to_tuple(), int(half_width))
        pygame.draw.circle(self.screen, base_color, p2.to_tuple(), int(half_width))
        
        # Edge outline for definition
        pygame.draw.polygon(self.screen, (0, 0, 0), points, 2)
    
    def draw_servo_motor(self, pos: Point2D, size: int, color: Tuple[int, int, int]):
        """Draw a servo motor joint"""
        # Main servo body (rectangle)
        servo_rect = pygame.Rect(
            int(pos.x - size),
            int(pos.y - size * 0.7),
            size * 2,
            size * 1.4
        )
        
        # Draw servo body with gradient effect
        pygame.draw.rect(self.screen, COLORS['joint'], servo_rect, border_radius=3)
        pygame.draw.rect(self.screen, COLORS['joint_light'], 
                        (servo_rect.x, servo_rect.y, servo_rect.width, servo_rect.height // 3),
                        border_radius=3)
        
        # Servo outline
        pygame.draw.rect(self.screen, (0, 0, 0), servo_rect, 2, border_radius=3)
        
        # Central shaft/axis
        pygame.draw.circle(self.screen, COLORS['metal'], pos.to_tuple(), int(size * 0.6))
        pygame.draw.circle(self.screen, COLORS['joint'], pos.to_tuple(), int(size * 0.5))
        pygame.draw.circle(self.screen, (0, 0, 0), pos.to_tuple(), int(size * 0.6), 2)
        
        # Bolts/screws on servo
        bolt_positions = [
            (pos.x - size * 0.7, pos.y - size * 0.4),
            (pos.x + size * 0.7, pos.y - size * 0.4),
            (pos.x - size * 0.7, pos.y + size * 0.4),
            (pos.x + size * 0.7, pos.y + size * 0.4)
        ]
        
        for bx, by in bolt_positions:
            pygame.draw.circle(self.screen, COLORS['bolt'], (int(bx), int(by)), 3)
            pygame.draw.circle(self.screen, (0, 0, 0), (int(bx), int(by)), 3, 1)
    
    def draw_foot(self, pos: Point2D, angle: float, color: Tuple[int, int, int],
                 dark_color: Tuple[int, int, int]):
        """Draw a robotic foot"""
        # Foot dimensions
        foot_length = 35
        foot_height = 12
        
        # Calculate foot orientation
        angle_rad = math.radians(angle)
        
        # Foot polygon (trapezoid shape)
        points = [
            (pos.x - foot_length * 0.3, pos.y - foot_height // 2),
            (pos.x + foot_length * 0.7, pos.y - foot_height // 2),
            (pos.x + foot_length * 0.5, pos.y + foot_height // 2),
            (pos.x - foot_length * 0.5, pos.y + foot_height // 2)
        ]
        
        # Draw foot with shading
        pygame.draw.polygon(self.screen, color, points)
        
        # Bottom sole (darker)
        sole_points = [
            points[2],
            points[3],
            (pos.x - foot_length * 0.5, pos.y + foot_height // 2 + 4),
            (pos.x + foot_length * 0.5, pos.y + foot_height // 2 + 4)
        ]
        pygame.draw.polygon(self.screen, dark_color, sole_points)
        
        # Outline
        pygame.draw.polygon(self.screen, (0, 0, 0), points, 2)
        
        # Tread marks on bottom
        for i in range(3):
            tx = pos.x - foot_length * 0.3 + i * foot_length * 0.3
            pygame.draw.line(self.screen, (0, 0, 0),
                           (tx, pos.y + foot_height // 2 + 2),
                           (tx, pos.y + foot_height // 2 + 4), 2)
    
    def draw_leg(self, segments: LegSegments, color: Tuple[int, int, int], label: str):
        """Draw one leg with realistic robotic appearance"""
        # Determine colors based on which leg
        if 'right' in label.lower():
            base_color = COLORS['right_leg']
            dark_color = COLORS['right_leg_dark']
            light_color = COLORS['right_leg_light']
        else:
            base_color = COLORS['left_leg']
            dark_color = COLORS['left_leg_dark']
            light_color = COLORS['left_leg_light']
        
        # Draw segments with 3D appearance (from back to front)
        # Thigh (hip to knee) - thicker
        self.draw_thick_segment(segments.hip, segments.knee, BONE_WIDTH + 4,
                               base_color, dark_color, light_color)
        
        # Shin (knee to ankle) - slightly thinner
        self.draw_thick_segment(segments.knee, segments.ankle, BONE_WIDTH,
                               base_color, dark_color, light_color)
        
        # Lower leg/foot connector (ankle to foot) - thinnest
        self.draw_thick_segment(segments.ankle, segments.foot, BONE_WIDTH - 4,
                               base_color, dark_color, light_color)
        
        # Draw servo motors at joints
        self.draw_servo_motor(segments.hip, JOINT_RADIUS, base_color)    # Hip servo
        self.draw_servo_motor(segments.knee, JOINT_RADIUS, base_color)   # Knee servo
        self.draw_servo_motor(segments.ankle, int(JOINT_RADIUS * 0.8), base_color)  # Ankle servo (smaller)
        
        # Draw robotic foot
        self.draw_foot(segments.foot, 0, base_color, dark_color)
        
        # Label with background
        label_surf = self.small_font.render(label, True, (255, 255, 255))
        label_bg = pygame.Surface((label_surf.get_width() + 8, label_surf.get_height() + 4))
        label_bg.set_alpha(200)
        label_bg.fill(base_color)
        self.screen.blit(label_bg, (segments.hip.x - 30, segments.hip.y - 30))
        self.screen.blit(label_surf, (segments.hip.x - 26, segments.hip.y - 28))
    
    def draw_angle_graph(self):
        """Draw angle history graph overlay"""
        if not self.show_graph:
            return
        
        graph_x = 20
        graph_y = 20
        graph_width = 300
        graph_height = 200
        
        # Semi-transparent background
        s = pygame.Surface((graph_width, graph_height))
        s.set_alpha(180)
        s.fill(COLORS['graph_bg'][:3])
        self.screen.blit(s, (graph_x, graph_y))
        
        # Border
        pygame.draw.rect(self.screen, COLORS['text'], 
                        (graph_x, graph_y, graph_width, graph_height), 1)
        
        # Title
        title = self.font.render("Angle History", True, COLORS['text'])
        self.screen.blit(title, (graph_x + 10, graph_y + 5))
        
        # Plot each angle
        colors = {
            'right_hip': (255, 100, 100),
            'right_knee': (255, 200, 100),
            'right_ankle': (255, 100, 200),
            'left_hip': (100, 100, 255),
            'left_knee': (100, 200, 255),
            'left_ankle': (200, 100, 255)
        }
        
        y_offset = 30
        for name, history in self.angle_history.items():
            if len(history) < 2:
                continue
            
            # Normalize to 0-180 range for display
            points = []
            for i, angle in enumerate(history):
                x = graph_x + 10 + (i * (graph_width - 20) / 100)
                y = graph_y + y_offset + graph_height - 40 - ((angle / 180.0) * (graph_height - 60))
                points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, colors[name], False, points, 1)
    
    def draw_hud(self):
        """Draw heads-up display with info"""
        angles = self.get_current_angles()
        
        info_lines = [
            f"Frame: {self.current_frame + 1}/{len(self.frames)}",
            f"Time: {angles.timestamp}",
            f"Speed: {self.speed_multiplier:.1f}x {'[PAUSED]' if not self.playing else ''}",
            f"",
            f"Right Leg: H:{angles.right_hip:.1f}° K:{angles.right_knee:.1f}° A:{angles.right_ankle:.1f}°",
            f"Left Leg:  H:{angles.left_hip:.1f}° K:{angles.left_knee:.1f}° A:{angles.left_ankle:.1f}°",
        ]
        
        y_offset = WINDOW_HEIGHT - 140
        for line in info_lines:
            if line:
                text_surf = self.small_font.render(line, True, COLORS['text'])
                self.screen.blit(text_surf, (10, y_offset))
            y_offset += 20
        
        # Controls
        controls = [
            "SPACE: Pause/Play  |  ←→: Step  |  ↑↓: Speed  |  R: Reset  |  G: Graph  |  Q: Quit"
        ]
        
        for i, line in enumerate(controls):
            text_surf = self.small_font.render(line, True, COLORS['text'])
            self.screen.blit(text_surf, (10, WINDOW_HEIGHT - 20))
    
    def update_angle_history(self):
        """Add current angles to history for graphing"""
        angles = self.get_current_angles()
        self.angle_history['right_hip'].append(angles.right_hip)
        self.angle_history['right_knee'].append(angles.right_knee)
        self.angle_history['right_ankle'].append(angles.right_ankle)
        self.angle_history['left_hip'].append(angles.left_hip)
        self.angle_history['left_knee'].append(angles.left_knee)
        self.angle_history['left_ankle'].append(angles.left_ankle)
    
    def render(self):
        """Render one frame"""
        self.screen.fill(COLORS['background'])
        
        # Draw environment
        self.draw_ground()
        
        # Draw torso and get hip positions
        right_hip_pos, left_hip_pos = self.draw_torso()
        
        # Get current angles
        angles = self.get_current_angles()
        
        # Calculate leg positions
        right_leg = calculate_leg_positions(
            right_hip_pos,
            angles.right_hip,
            angles.right_knee,
            angles.right_ankle
        )
        
        left_leg = calculate_leg_positions(
            left_hip_pos,
            angles.left_hip,
            angles.left_knee,
            angles.left_ankle
        )
        
        # Draw legs
        self.draw_leg(right_leg, COLORS['right_leg'], "RIGHT")
        self.draw_leg(left_leg, COLORS['left_leg'], "LEFT")
        
        # Draw UI
        self.draw_angle_graph()
        self.draw_hud()
        
        pygame.display.flip()
    
    def handle_events(self):
        """Handle keyboard and window events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    return False
                
                elif event.key == pygame.K_SPACE:
                    self.playing = not self.playing
                
                elif event.key == pygame.K_RIGHT:
                    self.current_frame = min(self.current_frame + 1, len(self.frames) - 1)
                    self.update_angle_history()
                
                elif event.key == pygame.K_LEFT:
                    self.current_frame = max(self.current_frame - 1, 0)
                
                elif event.key == pygame.K_r:
                    self.current_frame = 0
                    self.playing = False
                    for history in self.angle_history.values():
                        history.clear()
                
                elif event.key == pygame.K_UP:
                    self.speed_multiplier = min(self.speed_multiplier + 0.25, 5.0)
                
                elif event.key == pygame.K_DOWN:
                    self.speed_multiplier = max(self.speed_multiplier - 0.25, 0.25)
                
                elif event.key == pygame.K_g:
                    self.show_graph = not self.show_graph
        
        return True
    
    def update(self, dt: float):
        """Update simulation state"""
        if not self.playing:
            return
        
        # Calculate frame advance based on playback speed
        frame_duration = 1.0 / (self.playback_fps * self.speed_multiplier)
        self.frame_timer += dt
        
        if self.frame_timer >= frame_duration:
            self.frame_timer = 0.0
            self.current_frame += 1
            
            # Loop or stop at end
            if self.current_frame >= len(self.frames):
                self.current_frame = 0  # Loop
                # Alternative: stop at end
                # self.current_frame = len(self.frames) - 1
                # self.playing = False
            
            self.update_angle_history()
    
    def run(self):
        """Main simulation loop"""
        running = True
        
        print("\n=== Bionic Leg Simulator ===")
        print("Controls:")
        print("  SPACE     - Pause/Resume")
        print("  ← →       - Step frames")
        print("  ↑ ↓       - Speed up/down")
        print("  R         - Reset")
        print("  G         - Toggle angle graph")
        print("  Q/ESC     - Quit")
        print("\nStarting simulation...\n")
        
        while running:
            dt = self.clock.tick(FPS) / 1000.0  # Delta time in seconds
            
            running = self.handle_events()
            self.update(dt)
            self.render()
        
        pygame.quit()
        print("Simulation closed.")

# ============= 3D VISUALIZATION (OPTIONAL) =============

class Robot3DSimulator:
    """3D visualization using Pygame with pseudo-3D projection"""
    
    def __init__(self, csv_path: str, playback_fps: float = 30):
        print("3D visualization not yet implemented.")
        print("Using 2D simulator instead...")
        # Fall back to 2D for now
        self.simulator = RobotSimulator(csv_path, playback_fps)
    
    def run(self):
        self.simulator.run()

# ============= MAIN =============

def main():
    parser = argparse.ArgumentParser(
        description='Visual simulation of bionic leg movement from CSV data',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--csv', type=str, default='output_angles.csv',
                       help='Path to CSV file with joint angles')
    parser.add_argument('--fps', type=float, default=30,
                       help='Playback frame rate (default: 30)')
    parser.add_argument('--3d', action='store_true', dest='use_3d',
                       help='Use 3D visualization (experimental)')
    
    args = parser.parse_args()
    
    # Create and run simulator
    if args.use_3d:
        sim = Robot3DSimulator(args.csv, args.fps)
    else:
        sim = RobotSimulator(args.csv, args.fps)
    
    sim.run()

if __name__ == '__main__':
    main()
