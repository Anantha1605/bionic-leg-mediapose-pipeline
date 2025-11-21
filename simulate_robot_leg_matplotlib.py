#!/usr/bin/env python3
"""
simulate_robot_leg_matplotlib.py

Professional scientific visualization of bionic leg movement using Matplotlib.
Features publication-quality rendering with smooth animations and technical styling.

This version provides a more professional, research-oriented appearance compared
to the Pygame version, with better suited for:
- Scientific presentations
- Research papers
- Technical documentation
- Data analysis

Controls:
    SPACE     - Pause/Resume playback
    LEFT      - Step backward one frame
    RIGHT     - Step forward one frame
    R         - Reset to beginning
    Q/ESC     - Quit
    S         - Save current frame as image
    G         - Toggle angle plot panel

Usage:
    python simulate_robot_leg_matplotlib.py
    python simulate_robot_leg_matplotlib.py --csv output_angles.csv --fps 30
    python simulate_robot_leg_matplotlib.py --style dark

Author: Generated for bionic-leg-mediapose-pipeline
Date: 2025-11-18
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle, Polygon, FancyBboxPatch
from matplotlib.collections import LineCollection
import numpy as np
import csv
import argparse
import sys
from typing import List, Dict, Tuple
from dataclasses import dataclass
from collections import deque

# ============= CONFIGURATION =============

# Robot dimensions (in mm, will be scaled for display)
SEGMENT_LENGTHS = {
    'hip_to_knee': 60,   # Thigh length
    'knee_to_ankle': 70, # Shin length
    'ankle_to_foot': 40  # Foot length
}

# Visual settings
SCALE_FACTOR = 1.5  # Display scale
JOINT_SIZE = 12     # Servo motor size
SEGMENT_WIDTH = 8   # Leg segment width

# Animation settings
FPS = 30
FRAME_INTERVAL = 1000 / FPS  # milliseconds

# Style presets
STYLES = {
    'light': {
        'bg_color': '#F8F9FA',
        'grid_color': '#DEE2E6',
        'text_color': '#212529',
        'right_leg': '#DC3545',
        'left_leg': '#0D6EFD',
        'servo': '#495057',
        'torso': '#6C757D',
        'ground': '#ADB5BD'
    },
    'dark': {
        'bg_color': '#1A1D23',
        'grid_color': '#343A40',
        'text_color': '#F8F9FA',
        'right_leg': '#FF6B6B',
        'left_leg': '#4DABF7',
        'servo': '#868E96',
        'torso': '#ADB5BD',
        'ground': '#495057'
    },
    'scientific': {
        'bg_color': '#FFFFFF',
        'grid_color': '#E0E0E0',
        'text_color': '#000000',
        'right_leg': '#E63946',
        'left_leg': '#1D3557',
        'servo': '#2A9D8F',
        'torso': '#606060',
        'ground': '#CCCCCC'
    }
}

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
    """Calculate leg segment positions using forward kinematics"""
    
    # Convert angles to radians
    hip_rad = np.radians(180 - hip_angle)
    
    # Thigh segment (hip to knee)
    thigh_len = SEGMENT_LENGTHS['hip_to_knee'] * SCALE_FACTOR
    knee_x = hip_pos.x + thigh_len * np.sin(hip_rad)
    knee_y = hip_pos.y + thigh_len * np.cos(hip_rad)
    knee_pos = Point2D(knee_x, knee_y)
    
    # Shin segment (knee to ankle)
    shin_len = SEGMENT_LENGTHS['knee_to_ankle'] * SCALE_FACTOR
    knee_bend = np.radians(180 - knee_angle)
    shin_angle = hip_rad + knee_bend
    
    ankle_x = knee_x + shin_len * np.sin(shin_angle)
    ankle_y = knee_y + shin_len * np.cos(shin_angle)
    ankle_pos = Point2D(ankle_x, ankle_y)
    
    # Foot segment (ankle to foot tip)
    foot_len = SEGMENT_LENGTHS['ankle_to_foot'] * SCALE_FACTOR
    ankle_bend = np.radians(ankle_angle - 90)
    foot_angle = shin_angle + ankle_bend
    
    foot_x = ankle_x + foot_len * np.cos(foot_angle)
    foot_y = ankle_y + foot_len * np.sin(foot_angle)
    foot_pos = Point2D(foot_x, foot_y)
    
    return LegSegments(hip_pos, knee_pos, ankle_pos, foot_pos)

# ============= MATPLOTLIB ROBOT SIMULATOR =============

class MatplotlibRobotSimulator:
    """Professional robot simulator using Matplotlib"""
    
    def __init__(self, csv_path: str, playback_fps: float = 30, style: str = 'dark',
                 show_angles: bool = True):
        
        # Load data
        self.frames = load_csv_data(csv_path)
        if not self.frames:
            print("Error: No frames loaded")
            sys.exit(1)
        
        # Playback state
        self.current_frame = 0
        self.playing = True
        self.playback_fps = playback_fps
        self.speed_multiplier = 1.0
        self.show_angles = show_angles
        
        # Style
        self.style = STYLES.get(style, STYLES['dark'])
        
        # Angle history for plotting
        self.angle_history = {
            'frames': deque(maxlen=100),
            'right_hip': deque(maxlen=100),
            'right_knee': deque(maxlen=100),
            'right_ankle': deque(maxlen=100),
            'left_hip': deque(maxlen=100),
            'left_knee': deque(maxlen=100),
            'left_ankle': deque(maxlen=100)
        }
        
        # Setup figure
        self.setup_figure()
        
        # Animation
        self.anim = None
        
        # Torso position
        self.torso_x = 0
        self.torso_y = 0
    
    def setup_figure(self):
        """Setup matplotlib figure and axes"""
        plt.style.use('seaborn-v0_8-darkgrid' if 'dark' in str(self.style) else 'seaborn-v0_8-whitegrid')
        
        if self.show_angles:
            self.fig = plt.figure(figsize=(16, 9))
            gs = self.fig.add_gridspec(2, 2, width_ratios=[2, 1], height_ratios=[3, 1],
                                      hspace=0.3, wspace=0.3)
            
            # Main robot view
            self.ax_main = self.fig.add_subplot(gs[:, 0])
            
            # Angle plots
            self.ax_angles_r = self.fig.add_subplot(gs[0, 1])
            self.ax_angles_l = self.fig.add_subplot(gs[1, 1])
        else:
            self.fig, self.ax_main = plt.subplots(figsize=(12, 9))
        
        # Configure main axis
        self.ax_main.set_xlim(-150, 150)
        self.ax_main.set_ylim(-300, 50)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3, color=self.style['grid_color'])
        self.ax_main.set_facecolor(self.style['bg_color'])
        self.ax_main.set_xlabel('X Position (mm)', fontsize=10, color=self.style['text_color'])
        self.ax_main.set_ylabel('Y Position (mm)', fontsize=10, color=self.style['text_color'])
        self.ax_main.tick_params(colors=self.style['text_color'])
        
        # Configure angle plots if shown
        if self.show_angles:
            for ax, title in [(self.ax_angles_r, 'Right Leg Angles'), 
                             (self.ax_angles_l, 'Left Leg Angles')]:
                ax.set_facecolor(self.style['bg_color'])
                ax.grid(True, alpha=0.3, color=self.style['grid_color'])
                ax.set_xlabel('Frame', fontsize=8, color=self.style['text_color'])
                ax.set_ylabel('Angle (°)', fontsize=8, color=self.style['text_color'])
                ax.set_title(title, fontsize=10, color=self.style['text_color'], pad=10)
                ax.tick_params(colors=self.style['text_color'], labelsize=8)
                ax.set_ylim(0, 200)
        
        self.fig.patch.set_facecolor(self.style['bg_color'])
        
        # Title
        self.title = self.fig.suptitle('Bionic Leg Gait Analysis - MediaPose Data',
                                       fontsize=14, color=self.style['text_color'],
                                       fontweight='bold')
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
    
    def draw_thick_line(self, ax, p1: Point2D, p2: Point2D, color: str, width: float, 
                       alpha: float = 0.8, zorder: int = 2):
        """Draw a thick line segment with rounded caps"""
        line = ax.plot([p1.x, p2.x], [p1.y, p2.y], 
                      color=color, linewidth=width, 
                      solid_capstyle='round', solid_joinstyle='round',
                      alpha=alpha, zorder=zorder)[0]
        return line
    
    def draw_servo_motor(self, ax, pos: Point2D, size: float, color: str, zorder: int = 3):
        """Draw a servo motor joint"""
        # Servo body (rectangle)
        servo = FancyBboxPatch(
            (pos.x - size, pos.y - size * 0.7),
            size * 2, size * 1.4,
            boxstyle="round,pad=0.05",
            facecolor=self.style['servo'],
            edgecolor='black',
            linewidth=1.5,
            alpha=0.9,
            zorder=zorder
        )
        ax.add_patch(servo)
        
        # Central shaft
        shaft = Circle((pos.x, pos.y), size * 0.5, 
                      facecolor=color, edgecolor='black', 
                      linewidth=1, alpha=1.0, zorder=zorder+1)
        ax.add_patch(shaft)
        
        # Mounting bolts
        bolt_positions = [
            (pos.x - size * 0.6, pos.y - size * 0.4),
            (pos.x + size * 0.6, pos.y - size * 0.4),
            (pos.x - size * 0.6, pos.y + size * 0.4),
            (pos.x + size * 0.6, pos.y + size * 0.4)
        ]
        
        for bx, by in bolt_positions:
            bolt = Circle((bx, by), size * 0.15, 
                         facecolor='#2C3E50', edgecolor='black',
                         linewidth=0.5, zorder=zorder+1)
            ax.add_patch(bolt)
    
    def draw_foot(self, ax, pos: Point2D, color: str, zorder: int = 2):
        """Draw a robotic foot"""
        # Foot polygon (trapezoid)
        foot_length = 30
        foot_height = 10
        
        vertices = [
            [pos.x - foot_length * 0.3, pos.y - foot_height / 2],
            [pos.x + foot_length * 0.7, pos.y - foot_height / 2],
            [pos.x + foot_length * 0.5, pos.y + foot_height / 2],
            [pos.x - foot_length * 0.5, pos.y + foot_height / 2]
        ]
        
        foot = Polygon(vertices, facecolor=color, edgecolor='black',
                      linewidth=2, alpha=0.9, zorder=zorder)
        ax.add_patch(foot)
        
        # Tread marks
        for i in range(3):
            tx = pos.x - foot_length * 0.2 + i * foot_length * 0.3
            ax.plot([tx, tx], [pos.y + foot_height/2 - 2, pos.y + foot_height/2],
                   color='black', linewidth=1.5, zorder=zorder+1)
    
    def draw_torso(self, ax):
        """Draw robot torso"""
        torso_width = 50
        torso_height = 70
        
        # Main torso body
        torso = FancyBboxPatch(
            (-torso_width / 2, -torso_height),
            torso_width, torso_height,
            boxstyle="round,pad=0.1",
            facecolor=self.style['torso'],
            edgecolor='black',
            linewidth=2,
            alpha=0.8,
            zorder=1
        )
        ax.add_patch(torso)
        
        # Control panel
        panel = Rectangle(
            (-torso_width / 2 + 8, -torso_height + 10),
            torso_width - 16, 20,
            facecolor='#34495E',
            edgecolor='black',
            linewidth=1,
            zorder=2
        )
        ax.add_patch(panel)
        
        # LED indicators
        led_colors = ['#2ECC71', '#F39C12', '#3498DB']
        for i, led_color in enumerate(led_colors):
            led_x = -torso_width / 2 + 15 + i * 12
            led_y = -torso_height + 20
            led = Circle((led_x, led_y), 2.5, facecolor=led_color, 
                        edgecolor='black', linewidth=0.5, zorder=3)
            ax.add_patch(led)
        
        # Hip attachment points
        right_hip_x = 15
        left_hip_x = -15
        hip_y = 0
        
        for hip_x in [right_hip_x, left_hip_x]:
            bracket = Rectangle(
                (hip_x - 6, hip_y - 6),
                12, 12,
                facecolor='#95A5A6',
                edgecolor='black',
                linewidth=1.5,
                zorder=2
            )
            ax.add_patch(bracket)
        
        return Point2D(right_hip_x, hip_y), Point2D(left_hip_x, hip_y)
    
    def draw_ground(self, ax):
        """Draw ground platform"""
        ground_y = -280
        
        # Platform
        platform = Rectangle(
            (-200, ground_y),
            400, 15,
            facecolor=self.style['ground'],
            edgecolor='black',
            linewidth=2,
            alpha=0.7,
            zorder=0
        )
        ax.add_patch(platform)
        
        # Grid on platform
        for x in np.arange(-200, 200, 30):
            ax.plot([x, x], [ground_y, ground_y + 15],
                   color='black', linewidth=0.5, alpha=0.3, zorder=1)
    
    def draw_leg(self, ax, segments: LegSegments, color: str, label: str):
        """Draw one leg with professional appearance"""
        # Draw segments (thigh, shin, lower leg)
        self.draw_thick_line(ax, segments.hip, segments.knee, color, SEGMENT_WIDTH + 3, zorder=2)
        self.draw_thick_line(ax, segments.knee, segments.ankle, color, SEGMENT_WIDTH, zorder=2)
        self.draw_thick_line(ax, segments.ankle, segments.foot, color, SEGMENT_WIDTH - 3, zorder=2)
        
        # Draw servo motors at joints
        self.draw_servo_motor(ax, segments.hip, JOINT_SIZE, color, zorder=3)
        self.draw_servo_motor(ax, segments.knee, JOINT_SIZE, color, zorder=3)
        self.draw_servo_motor(ax, segments.ankle, JOINT_SIZE * 0.8, color, zorder=3)
        
        # Draw foot
        self.draw_foot(ax, segments.foot, color, zorder=2)
        
        # Label
        ax.text(segments.hip.x, segments.hip.y + 20, label,
               fontsize=9, color='white', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor=color, alpha=0.8),
               ha='center', zorder=5)
    
    def update_angle_plots(self):
        """Update angle history plots"""
        if not self.show_angles:
            return
        
        angles = self.frames[self.current_frame]
        
        # Add to history
        self.angle_history['frames'].append(angles.frame)
        self.angle_history['right_hip'].append(angles.right_hip)
        self.angle_history['right_knee'].append(angles.right_knee)
        self.angle_history['right_ankle'].append(angles.right_ankle)
        self.angle_history['left_hip'].append(angles.left_hip)
        self.angle_history['left_knee'].append(angles.left_knee)
        self.angle_history['left_ankle'].append(angles.left_ankle)
        
        # Clear and redraw
        self.ax_angles_r.clear()
        self.ax_angles_l.clear()
        
        frames = list(self.angle_history['frames'])
        
        # Right leg plot
        self.ax_angles_r.plot(frames, self.angle_history['right_hip'], 
                             label='Hip', color='#E74C3C', linewidth=2, marker='o', markersize=3)
        self.ax_angles_r.plot(frames, self.angle_history['right_knee'], 
                             label='Knee', color='#F39C12', linewidth=2, marker='s', markersize=3)
        self.ax_angles_r.plot(frames, self.angle_history['right_ankle'], 
                             label='Ankle', color='#E67E22', linewidth=2, marker='^', markersize=3)
        
        # Left leg plot
        self.ax_angles_l.plot(frames, self.angle_history['left_hip'], 
                             label='Hip', color='#3498DB', linewidth=2, marker='o', markersize=3)
        self.ax_angles_l.plot(frames, self.angle_history['left_knee'], 
                             label='Knee', color='#2ECC71', linewidth=2, marker='s', markersize=3)
        self.ax_angles_l.plot(frames, self.angle_history['left_ankle'], 
                             label='Ankle', color='#1ABC9C', linewidth=2, marker='^', markersize=3)
        
        # Reconfigure
        for ax, title in [(self.ax_angles_r, 'Right Leg Angles'), 
                         (self.ax_angles_l, 'Left Leg Angles')]:
            ax.set_facecolor(self.style['bg_color'])
            ax.grid(True, alpha=0.3, color=self.style['grid_color'])
            ax.set_xlabel('Frame', fontsize=8, color=self.style['text_color'])
            ax.set_ylabel('Angle (°)', fontsize=8, color=self.style['text_color'])
            ax.set_title(title, fontsize=10, color=self.style['text_color'], pad=10)
            ax.tick_params(colors=self.style['text_color'], labelsize=8)
            ax.set_ylim(0, 200)
            ax.legend(loc='upper right', fontsize=7, framealpha=0.9)
    
    def update_frame(self, frame_num):
        """Update animation frame"""
        if not self.playing:
            return
        
        self.current_frame = (self.current_frame + 1) % len(self.frames)
        self.render()
    
    def render(self):
        """Render current frame"""
        # Clear main axis
        self.ax_main.clear()
        
        # Reconfigure main axis
        self.ax_main.set_xlim(-150, 150)
        self.ax_main.set_ylim(-300, 50)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3, color=self.style['grid_color'])
        self.ax_main.set_facecolor(self.style['bg_color'])
        self.ax_main.set_xlabel('X Position (mm)', fontsize=10, color=self.style['text_color'])
        self.ax_main.set_ylabel('Y Position (mm)', fontsize=10, color=self.style['text_color'])
        self.ax_main.tick_params(colors=self.style['text_color'])
        
        # Draw ground
        self.draw_ground(self.ax_main)
        
        # Draw torso
        right_hip_pos, left_hip_pos = self.draw_torso(self.ax_main)
        
        # Get current angles
        angles = self.frames[self.current_frame]
        
        # Calculate leg positions
        right_leg = calculate_leg_positions(right_hip_pos, angles.right_hip, 
                                           angles.right_knee, angles.right_ankle)
        left_leg = calculate_leg_positions(left_hip_pos, angles.left_hip, 
                                          angles.left_knee, angles.left_ankle)
        
        # Draw legs
        self.draw_leg(self.ax_main, right_leg, self.style['right_leg'], 'RIGHT')
        self.draw_leg(self.ax_main, left_leg, self.style['left_leg'], 'LEFT')
        
        # Info text
        info_text = (f"Frame: {self.current_frame + 1}/{len(self.frames)} | "
                    f"Time: {angles.timestamp} | "
                    f"Speed: {self.speed_multiplier:.1f}x"
                    f"{' [PAUSED]' if not self.playing else ''}")
        
        self.ax_main.text(0, 40, info_text,
                         fontsize=9, ha='center', color=self.style['text_color'],
                         bbox=dict(boxstyle='round,pad=0.5', facecolor=self.style['bg_color'],
                                  edgecolor=self.style['text_color'], alpha=0.8))
        
        # Controls hint
        controls = "SPACE: Pause | ←→: Step | R: Reset | S: Save | Q: Quit | G: Toggle Angles"
        self.ax_main.text(0, -290, controls,
                         fontsize=7, ha='center', color=self.style['text_color'],
                         alpha=0.7)
        
        # Update angle plots
        self.update_angle_plots()
        
        self.fig.canvas.draw()
    
    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == ' ':  # Space
            self.playing = not self.playing
            self.render()
        
        elif event.key == 'right':
            self.current_frame = min(self.current_frame + 1, len(self.frames) - 1)
            self.render()
        
        elif event.key == 'left':
            self.current_frame = max(self.current_frame - 1, 0)
            self.render()
        
        elif event.key == 'r':
            self.current_frame = 0
            self.playing = False
            self.angle_history = {k: deque(maxlen=100) for k in self.angle_history.keys()}
            self.render()
        
        elif event.key == 's':
            filename = f'robot_frame_{self.current_frame:04d}.png'
            self.fig.savefig(filename, dpi=300, bbox_inches='tight', 
                           facecolor=self.style['bg_color'])
            print(f"Saved: {filename}")
        
        elif event.key == 'g':
            self.show_angles = not self.show_angles
            self.setup_figure()
            self.render()
        
        elif event.key == 'q' or event.key == 'escape':
            plt.close(self.fig)
    
    def run(self):
        """Start the simulation"""
        print("\n=== Matplotlib Bionic Leg Simulator (Professional) ===")
        print("Controls:")
        print("  SPACE     - Pause/Resume")
        print("  ← →       - Step frames")
        print("  R         - Reset")
        print("  S         - Save frame as PNG")
        print("  G         - Toggle angle plots")
        print("  Q/ESC     - Quit")
        print("\nStarting simulation...\n")
        
        # Initial render
        self.render()
        
        # Setup animation
        self.anim = animation.FuncAnimation(
            self.fig, self.update_frame,
            interval=FRAME_INTERVAL / self.speed_multiplier,
            blit=False,
            cache_frame_data=False
        )
        
        plt.show()

# ============= MAIN =============

def main():
    parser = argparse.ArgumentParser(
        description='Professional scientific visualization of bionic leg movement',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--csv', type=str, default='output_angles.csv',
                       help='Path to CSV file with joint angles')
    parser.add_argument('--fps', type=float, default=30,
                       help='Playback frame rate (default: 30)')
    parser.add_argument('--style', type=str, choices=['light', 'dark', 'scientific'],
                       default='dark', help='Visual style (default: dark)')
    parser.add_argument('--no-angles', action='store_true',
                       help='Hide angle plot panels')
    
    args = parser.parse_args()
    
    # Create and run simulator
    sim = MatplotlibRobotSimulator(
        args.csv, 
        args.fps, 
        args.style,
        show_angles=not args.no_angles
    )
    sim.run()

if __name__ == '__main__':
    main()
