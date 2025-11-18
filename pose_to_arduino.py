#!/usr/bin/env python3
"""
pose_to_arduino.py

Advanced CSV-to-Arduino streamer for bionic leg gait replication with:
- Low-pass filtering for smooth motion
- Frame interpolation for consistent timing
- Relative movement mode
- Serial communication with Arduino
- Configurable frame rate
- Baseline reference tracking
- Emergency stop handling

Usage:
    python pose_to_arduino.py --port COM3 --csv output_angles.csv --rate 50
    python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --baseline-frame 1
    python pose_to_arduino.py --port COM3 --csv output_angles.csv --dry-run

Options:
    --port PORT         Serial port (e.g., COM3, /dev/ttyUSB0)
    --csv FILE          CSV file with angle data (default: output_angles.csv)
    --rate HZ           Playback rate in Hz (default: 50)
    --baud RATE         Baud rate (default: 115200)
    --relative          Enable relative movement mode
    --baseline-frame N  Frame number to use as baseline (default: 1)
    --filter-alpha A    Low-pass filter alpha (0-1, default: 0.2)
    --dry-run           Print commands without sending
    --loop              Loop playback continuously
    --verbose           Verbose output

Author: Generated for bionic-leg-mediapose-pipeline
Date: 2025-11-18
"""

import csv
import argparse
import time
import sys
from typing import List, Dict, Optional
import math

try:
    import serial
    from serial.tools import list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("Warning: pyserial not installed. Install with: pip install pyserial")

# ============= CONFIGURATION =============

DEFAULT_CSV = 'output_angles.csv'
DEFAULT_BAUD = 115200
DEFAULT_RATE = 50  # Hz
DEFAULT_FILTER_ALPHA = 0.2

# Column names in CSV
ANGLE_COLUMNS = [
    'right_hip_angle',
    'right_knee_angle', 
    'right_ankle_angle',
    'left_hip_angle',
    'left_knee_angle',
    'left_ankle_angle'
]

# ============= HELPER CLASSES =============

class LowPassFilter:
    """Exponential moving average low-pass filter"""
    
    def __init__(self, alpha: float = 0.2, initial_value: float = 90.0):
        self.alpha = alpha
        self.value = initial_value
    
    def update(self, new_value: float) -> float:
        """Update filter with new value and return filtered output"""
        if math.isnan(new_value):
            return self.value
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self, value: float = 90.0):
        """Reset filter to specific value"""
        self.value = value


class FrameInterpolator:
    """Interpolate between CSV frames for smooth playback"""
    
    def __init__(self, target_rate: float = 50.0):
        self.target_rate = target_rate
        self.target_dt = 1.0 / target_rate
    
    def interpolate(self, frame1: Dict[str, float], frame2: Dict[str, float], 
                   fraction: float) -> Dict[str, float]:
        """Linear interpolation between two frames"""
        result = {}
        for key in ANGLE_COLUMNS:
            v1 = frame1.get(key, 90.0)
            v2 = frame2.get(key, 90.0)
            
            # Handle NaN
            if math.isnan(v1): v1 = 90.0
            if math.isnan(v2): v2 = 90.0
            
            result[key] = v1 + (v2 - v1) * fraction
        
        return result


class ServoController:
    """Serial communication with Arduino servo controller"""
    
    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        if not SERIAL_AVAILABLE:
            raise RuntimeError("pyserial not installed")
        
        self.port = port
        self.baud = baud
        self.serial = None
        self.connected = False
        
        try:
            self.serial = serial.Serial(port, baud, timeout=timeout)
            time.sleep(2.0)  # Wait for Arduino reset
            self.serial.flushInput()
            self.connected = True
            print(f"Connected to {port} at {baud} baud")
        except Exception as e:
            print(f"Error connecting to {port}: {e}")
            raise
    
    def send_command(self, command: str, wait_response: bool = True) -> Optional[str]:
        """Send command to Arduino and optionally wait for response"""
        if not self.connected:
            return None
        
        try:
            self.serial.write(f"{command}\n".encode('utf-8'))
            self.serial.flush()
            
            if wait_response:
                response = self.serial.readline().decode('utf-8', errors='ignore').strip()
                return response
            return None
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
    
    def set_angles(self, angles: List[float]) -> bool:
        """Send ANGLES command with 6 joint angles"""
        if len(angles) != 6:
            print(f"Error: Expected 6 angles, got {len(angles)}")
            return False
        
        # Format: ANGLES,r_hip,r_knee,r_ankle,l_hip,l_knee,l_ankle
        angle_str = ','.join([f"{a:.2f}" for a in angles])
        cmd = f"ANGLES,{angle_str}"
        
        response = self.send_command(cmd, wait_response=True)
        return response == "OK"
    
    def set_baseline(self, angles: List[float]) -> bool:
        """Send BASELINE command"""
        if len(angles) != 6:
            return False
        
        angle_str = ','.join([f"{a:.2f}" for a in angles])
        cmd = f"BASELINE,{angle_str}"
        
        response = self.send_command(cmd, wait_response=True)
        return response == "OK"
    
    def set_relative_mode(self, enabled: bool) -> bool:
        """Enable/disable relative movement mode"""
        cmd = f"RELATIVE,{1 if enabled else 0}"
        response = self.send_command(cmd, wait_response=True)
        return response is not None
    
    def emergency_stop(self) -> bool:
        """Send emergency stop command"""
        response = self.send_command("STOP", wait_response=True)
        return response == "OK"
    
    def center_all(self) -> bool:
        """Center all servos"""
        response = self.send_command("CENTER", wait_response=True)
        return response is not None
    
    def get_status(self) -> Optional[str]:
        """Get controller status"""
        return self.send_command("STATUS", wait_response=True)
    
    def close(self):
        """Close serial connection"""
        if self.serial:
            self.serial.close()
            self.connected = False


# ============= CSV PROCESSING =============

def load_csv_frames(csv_path: str) -> List[Dict[str, float]]:
    """Load all frames from CSV file"""
    frames = []
    
    try:
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            
            for row in reader:
                frame_data = {}
                
                # Extract angles
                for col in ANGLE_COLUMNS:
                    try:
                        value = float(row.get(col, '90.0'))
                        frame_data[col] = value
                    except (ValueError, TypeError):
                        frame_data[col] = 90.0  # Default fallback
                
                # Store frame number and timestamp if available
                frame_data['frame'] = int(row.get('frame', len(frames) + 1))
                frame_data['timestamp'] = row.get('timestamp', '')
                
                frames.append(frame_data)
        
        print(f"Loaded {len(frames)} frames from {csv_path}")
        return frames
    
    except FileNotFoundError:
        print(f"Error: CSV file not found: {csv_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading CSV: {e}")
        sys.exit(1)


def extract_angles(frame: Dict[str, float]) -> List[float]:
    """Extract 6 angles from frame in correct order"""
    return [
        frame.get('right_hip_angle', 90.0),
        frame.get('right_knee_angle', 90.0),
        frame.get('right_ankle_angle', 90.0),
        frame.get('left_hip_angle', 90.0),
        frame.get('left_knee_angle', 90.0),
        frame.get('left_ankle_angle', 90.0)
    ]


# ============= MAIN STREAMER =============

def stream_csv_to_arduino(args):
    """Main streaming function"""
    
    # Load CSV frames
    frames = load_csv_frames(args.csv)
    
    if len(frames) == 0:
        print("Error: No frames loaded")
        return
    
    # Initialize filters
    filters = [LowPassFilter(alpha=args.filter_alpha) for _ in range(6)]
    
    # Initialize interpolator
    interpolator = FrameInterpolator(target_rate=args.rate)
    
    # Initialize controller
    controller = None
    if not args.dry_run:
        try:
            controller = ServoController(args.port, args.baud)
            
            # Set relative mode if requested
            if args.relative:
                controller.set_relative_mode(True)
                print("Relative movement mode enabled")
                
                # Set baseline from specified frame
                baseline_frame = frames[min(args.baseline_frame - 1, len(frames) - 1)]
                baseline_angles = extract_angles(baseline_frame)
                controller.set_baseline(baseline_angles)
                print(f"Baseline set from frame {args.baseline_frame}")
            else:
                controller.set_relative_mode(False)
                print("Absolute movement mode")
            
        except Exception as e:
            print(f"Failed to initialize controller: {e}")
            return
    
    # Calculate timing
    dt = 1.0 / args.rate
    print(f"\nStreaming at {args.rate} Hz ({dt*1000:.1f}ms per frame)")
    print("Press Ctrl+C to stop\n")
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            if args.verbose:
                print(f"\n=== Loop {loop_count} ===")
            
            frame_count = 0
            start_time = time.time()
            
            for i in range(len(frames)):
                frame = frames[i]
                
                # Extract raw angles
                raw_angles = extract_angles(frame)
                
                # Apply low-pass filtering
                filtered_angles = [
                    filters[j].update(raw_angles[j]) for j in range(6)
                ]
                
                # Send to controller or print
                if args.dry_run:
                    angle_str = ','.join([f"{a:.2f}" for a in filtered_angles])
                    print(f"Frame {frame['frame']:3d}: ANGLES,{angle_str}")
                else:
                    if controller:
                        success = controller.set_angles(filtered_angles)
                        if not success:
                            print(f"Warning: Failed to set angles for frame {frame['frame']}")
                        
                        if args.verbose:
                            print(f"Frame {frame['frame']:3d}: Sent - " + 
                                  ' '.join([f"{a:6.2f}" for a in filtered_angles]))
                
                frame_count += 1
                
                # Timing control
                elapsed = time.time() - start_time
                expected = frame_count * dt
                sleep_time = expected - elapsed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.1 and args.verbose:
                    print(f"Warning: Running {-sleep_time*1000:.1f}ms behind")
            
            # Loop control
            if not args.loop:
                break
            
            # Reset filters for next loop
            for f in filters:
                f.reset()
            
            print(f"\nLoop {loop_count} completed. {frame_count} frames sent.")
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        # Clean shutdown
        if controller:
            print("Sending emergency stop...")
            controller.emergency_stop()
            time.sleep(0.5)
            controller.close()
            print("Connection closed")


# ============= CLI =============

def list_serial_ports():
    """List available serial ports"""
    if not SERIAL_AVAILABLE:
        print("pyserial not installed")
        return []
    
    ports = list_ports.comports()
    return [p.device for p in ports]


def main():
    parser = argparse.ArgumentParser(
        description='Stream CSV angle data to Arduino servo controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument('--port', type=str, help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--csv', type=str, default=DEFAULT_CSV, help=f'CSV file path (default: {DEFAULT_CSV})')
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE, help=f'Playback rate in Hz (default: {DEFAULT_RATE})')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help=f'Baud rate (default: {DEFAULT_BAUD})')
    parser.add_argument('--relative', action='store_true', help='Enable relative movement mode')
    parser.add_argument('--baseline-frame', type=int, default=1, help='Frame to use as baseline (default: 1)')
    parser.add_argument('--filter-alpha', type=float, default=DEFAULT_FILTER_ALPHA, 
                       help=f'Low-pass filter alpha 0-1 (default: {DEFAULT_FILTER_ALPHA})')
    parser.add_argument('--dry-run', action='store_true', help='Print commands without sending to Arduino')
    parser.add_argument('--loop', action='store_true', help='Loop playback continuously')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--list-ports', action='store_true', help='List available serial ports and exit')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list_ports:
        ports = list_serial_ports()
        if ports:
            print("Available serial ports:")
            for p in ports:
                print(f"  {p}")
        else:
            print("No serial ports found")
        return
    
    # Validate arguments
    if not args.dry_run and not args.port:
        print("Error: --port required (or use --dry-run)")
        print("\nAvailable ports:")
        for p in list_serial_ports():
            print(f"  {p}")
        sys.exit(1)
    
    if args.filter_alpha < 0 or args.filter_alpha > 1:
        print("Error: --filter-alpha must be between 0 and 1")
        sys.exit(1)
    
    # Run streamer
    stream_csv_to_arduino(args)


if __name__ == '__main__':
    main()
