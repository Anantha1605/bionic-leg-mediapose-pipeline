#!/usr/bin/env python3
"""
send_csv_to_arduino.py - SLOW & SAFE VERSION (SYNCHRONOUS)

Reads `output_angles.csv` and streams angles to Arduino row-by-row.
Waits for Arduino to COMPLETE each movement before sending next row.

CRITICAL CHANGES:
- SYNCHRONOUS EXECUTION: Next row only after Arduino confirms completion
- Default FPS: 2 (very slow, 500ms per frame)
- Angle clamping to safe ranges (prevents over-rotation)
- Smooth transitions with exponential filtering
- Visual progress feedback

Usage:
  python send_csv_to_arduino.py --port COM3                      # 500ms/frame (default)
  python send_csv_to_arduino.py --port COM3 --min-delay 1000    # 1 second per frame
  python send_csv_to_arduino.py --port COM3 --min-delay 300     # 300ms per frame (faster)
  python send_csv_to_arduino.py --port COM3 --fps 5 --smooth    # 5 FPS with smoothing
  python send_csv_to_arduino.py --dry-run                       # Test without hardware

Options:
  --port COMx       Serial port (required unless --dry-run)
  --baud N          Baud rate (default 115200)
  --fps N           Frames per second (default 2 - VERY SLOW)
  --min-delay N     Minimum milliseconds per frame (overrides --fps)
  --smooth          Enable angle smoothing/filtering
  --smooth-factor   Smoothing strength 0.1-1.0 (default 0.3, lower=smoother)
  --angle-min       Minimum safe angle (default 60)
  --angle-max       Maximum safe angle (default 120)
  --dry-run         Print commands without sending
  --skip-empty      Skip rows with missing angles
  --center-first    Send CENTER command before starting
"""

import csv
import argparse
import time
import sys

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    print("WARNING: pyserial not installed. Use --dry-run mode or install: pip install pyserial")

CSV_PATH = 'output_angles.csv'

# ==========================================
# SAFE ANGLE RANGES (CRITICAL!)
# ==========================================
# Conservative ranges to prevent mechanical damage
# Adjust these after careful calibration!
DEFAULT_ANGLE_MIN = 60   # Degrees
DEFAULT_ANGLE_MAX = 120  # Degrees
DEFAULT_NEUTRAL = 90     # Safe neutral position


def find_ports():
    """List available serial ports"""
    if serial is None:
        return []
    return [p.device for p in list_ports.comports()]


def clamp_angle(angle, min_angle, max_angle, neutral):
    """
    Clamp angle to safe range
    Returns: (clamped_value, was_clamped)
    """
    try:
        val = float(angle)
    except (ValueError, TypeError):
        return neutral, True
    
    if val < min_angle:
        return min_angle, True
    elif val > max_angle:
        return max_angle, True
    else:
        return val, False


def smooth_transition(prev_val, new_val, smoothing_factor):
    """
    Apply exponential smoothing: output = prev + factor * (new - prev)
    smoothing_factor: 0.1 (very smooth) to 1.0 (instant)
    """
    if prev_val is None:
        return new_val
    return prev_val + smoothing_factor * (new_val - prev_val)


def build_angles_command(rhip, rknee, rankle, lhip, lknee, lankle):
    """
    Build ANGLES command matching Arduino protocol:
    ANGLES,<r_hip>,<r_knee>,<r_ankle>,<l_hip>,<l_knee>,<l_ankle>
    
    Preserves 2 decimal places for smooth motion
    """
    def fmt(val):
        try:
            return f"{float(val):.2f}"
        except (ValueError, TypeError):
            return "90.00"  # Safe fallback
    
    return f"ANGLES,{fmt(rhip)},{fmt(rknee)},{fmt(rankle)},{fmt(lhip)},{fmt(lknee)},{fmt(lankle)}\n"


def open_serial(port, baud, timeout=2):
    """Open serial connection with Arduino"""
    if serial is None:
        raise RuntimeError('pyserial not installed. Run: pip install pyserial')
    
    print(f"Opening {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=timeout)
    
    # Wait for Arduino reset (if auto-reset enabled)
    print("Waiting for Arduino to initialize...")
    time.sleep(2.5)
    
    # Clear any startup messages
    ser.flushInput()
    ser.flushOutput()
    
    # Wait for READY signal
    start_time = time.time()
    while time.time() - start_time < 5:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino: {line}")
            if "READY" in line:
                break
    
    return ser


def send_command_and_wait_completion(ser, cmd, timeout=10.0, max_retries=3):
    """
    Send command and wait for COMPLETE response (synchronous execution)
    Returns: (success, response_text, completion_time)
    """
    for attempt in range(max_retries):
        try:
            ser.write(cmd.encode('utf-8'))
            ser.flush()
            
            start_wait = time.time()
            
            # Wait for ACK (OK) first
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if response.startswith("ERR_"):
                return False, response, 0.0
            
            if response != "OK":
                if attempt < max_retries - 1:
                    print(f"  Retry {attempt + 1}/{max_retries}: got '{response}' instead of OK")
                    time.sleep(0.05)
                    continue
                else:
                    return False, f"No ACK: {response}", 0.0
            
            # Now wait for COMPLETE signal (servos reached target)
            while time.time() - start_wait < timeout:
                if ser.in_waiting:
                    complete_msg = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if complete_msg == "COMPLETE":
                        completion_time = time.time() - start_wait
                        return True, "COMPLETE", completion_time
                    elif complete_msg.startswith("ERR_"):
                        return False, complete_msg, 0.0
                
                time.sleep(0.01)  # Small delay to avoid busy-waiting
            
            # Timeout waiting for COMPLETE
            if attempt < max_retries - 1:
                print(f"  Retry {attempt + 1}/{max_retries}: timeout waiting for COMPLETE")
            else:
                return False, "Timeout waiting for COMPLETE", 0.0
        
        except serial.SerialException as e:
            print(f"  Serial error: {e}")
            if attempt < max_retries - 1:
                time.sleep(0.1)
            else:
                return False, str(e), 0.0
    
    return False, "Max retries exceeded", 0.0


def main():
    parser = argparse.ArgumentParser(
        description='Stream CSV joint angles to Arduino servo controller (SLOW & SAFE)',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--port', help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--fps', type=float, default=2.0, help='Playback FPS (default: 2 - VERY SLOW)')
    parser.add_argument('--min-delay', type=float, default=500.0, help='Minimum delay per frame in milliseconds (default: 500ms = 2 FPS)')
    parser.add_argument('--csv', default=CSV_PATH, help=f'CSV file path (default: {CSV_PATH})')
    parser.add_argument('--dry-run', action='store_true', help='Print commands without sending')
    parser.add_argument('--skip-empty', action='store_true', help='Skip rows with missing data')
    parser.add_argument('--smooth', action='store_true', help='Enable angle smoothing/filtering')
    parser.add_argument('--smooth-factor', type=float, default=0.3, help='Smoothing factor 0.1-1.0 (default: 0.3)')
    parser.add_argument('--angle-min', type=float, default=DEFAULT_ANGLE_MIN, help=f'Minimum safe angle (default: {DEFAULT_ANGLE_MIN})')
    parser.add_argument('--angle-max', type=float, default=DEFAULT_ANGLE_MAX, help=f'Maximum safe angle (default: {DEFAULT_ANGLE_MAX})')
    parser.add_argument('--limit', type=int, help='Limit number of frames to send (for testing)')
    parser.add_argument('--start-frame', type=int, default=0, help='Start from frame N')
    parser.add_argument('--verbose', '-v', action='store_true', help='Print each command sent')
    parser.add_argument('--center-first', action='store_true', help='Send CENTER command before starting')
    
    args = parser.parse_args()
    
    # Check port requirement
    if args.port is None and not args.dry_run:
        print('ERROR: --port required (or use --dry-run for testing)\n')
        print('Available serial ports:')
        ports = find_ports()
        if ports:
            for p in ports:
                print(f'  {p}')
        else:
            print('  No serial ports found')
        sys.exit(1)
    
    # Open CSV file
    try:
        csv_file = open(args.csv, 'r', newline='')
        print(f"Opened CSV: {args.csv}")
    except FileNotFoundError:
        print(f'ERROR: CSV file not found: {args.csv}')
        sys.exit(1)
    
    reader = csv.DictReader(csv_file)
    
    # Verify CSV columns
    required_cols = [
        'right_hip_angle', 'right_knee_angle', 'right_ankle_angle',
        'left_hip_angle', 'left_knee_angle', 'left_ankle_angle'
    ]
    
    first_row = next(reader, None)
    if first_row is None:
        print("ERROR: CSV file is empty")
        sys.exit(1)
    
    missing_cols = [col for col in required_cols if col not in first_row]
    if missing_cols:
        print(f"ERROR: CSV missing columns: {missing_cols}")
        sys.exit(1)
    
    # Reset reader
    csv_file.seek(0)
    next(reader)  # Skip header
    
    # Open serial connection
    ser = None
    if not args.dry_run:
        try:
            ser = open_serial(args.port, args.baud)
            print("✓ Connected to Arduino\n")
            
            # Send CENTER command if requested
            if args.center_first:
                print("Centering servos...")
                ser.write(b"CENTER\n")
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"  {response}")
                time.sleep(1)
            
        except Exception as e:
            print(f"ERROR: Failed to open serial port: {e}")
            sys.exit(1)
    
    # Calculate frame delay
    if args.min_delay:
        min_frame_delay = args.min_delay / 1000.0  # Convert ms to seconds
        calculated_fps = 1.0 / min_frame_delay if min_frame_delay > 0 else 0
    else:
        min_frame_delay = 1.0 / args.fps
        calculated_fps = args.fps
    
    print(f"⚠️  SLOW & SAFE MODE ⚠️")
    print(f"Minimum delay per frame: {min_frame_delay*1000:.0f}ms ({calculated_fps:.2f} FPS max)")
    print(f"Angle limits: {args.angle_min}° - {args.angle_max}°")
    if args.smooth:
        print(f"Smoothing: ENABLED (factor={args.smooth_factor})")
    else:
        print(f"Smoothing: DISABLED")
    print(f"⚠️  Each frame will take AT LEAST {min_frame_delay*1000:.0f}ms (servo completion + delay)")
    print()
    
    if args.dry_run:
        print("=== DRY RUN MODE (not sending to Arduino) ===\n")
    
    # Initialize smoothing state
    prev_angles = None  # [rhip, rknee, rankle, lhip, lknee, lankle]
    
    # Process CSV rows
    sent_count = 0
    error_count = 0
    skip_count = 0
    clamp_count = 0
    start_time = time.time()
    
    try:
        for frame_idx, row in enumerate(reader):
            # Skip to start frame
            if frame_idx < args.start_frame:
                continue
            
            # Check limit
            if args.limit and sent_count >= args.limit:
                print(f"\nReached frame limit ({args.limit})")
                break
            
            # Extract angles
            rhip = row.get('right_hip_angle', '').strip()
            rknee = row.get('right_knee_angle', '').strip()
            rankle = row.get('right_ankle_angle', '').strip()
            lhip = row.get('left_hip_angle', '').strip()
            lknee = row.get('left_knee_angle', '').strip()
            lankle = row.get('left_ankle_angle', '').strip()
            
            # Check for empty values
            if args.skip_empty:
                if not all([rhip, rknee, rankle, lhip, lknee, lankle]):
                    skip_count += 1
                    continue
            
            # Use defaults for missing values
            rhip = rhip or str(DEFAULT_NEUTRAL)
            rknee = rknee or str(DEFAULT_NEUTRAL)
            rankle = rankle or str(DEFAULT_NEUTRAL)
            lhip = lhip or str(DEFAULT_NEUTRAL)
            lknee = lknee or str(DEFAULT_NEUTRAL)
            lankle = lankle or str(DEFAULT_NEUTRAL)
            
            # Clamp angles to safe ranges
            rhip_val, rhip_clamped = clamp_angle(rhip, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            rknee_val, rknee_clamped = clamp_angle(rknee, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            rankle_val, rankle_clamped = clamp_angle(rankle, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            lhip_val, lhip_clamped = clamp_angle(lhip, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            lknee_val, lknee_clamped = clamp_angle(lknee, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            lankle_val, lankle_clamped = clamp_angle(lankle, args.angle_min, args.angle_max, DEFAULT_NEUTRAL)
            
            # Track clamping
            if any([rhip_clamped, rknee_clamped, rankle_clamped, lhip_clamped, lknee_clamped, lankle_clamped]):
                clamp_count += 1
                if args.verbose:
                    clamped_joints = []
                    if rhip_clamped: clamped_joints.append(f"R_Hip:{rhip}->{rhip_val:.1f}")
                    if rknee_clamped: clamped_joints.append(f"R_Knee:{rknee}->{rknee_val:.1f}")
                    if rankle_clamped: clamped_joints.append(f"R_Ankle:{rankle}->{rankle_val:.1f}")
                    if lhip_clamped: clamped_joints.append(f"L_Hip:{lhip}->{lhip_val:.1f}")
                    if lknee_clamped: clamped_joints.append(f"L_Knee:{lknee}->{lknee_val:.1f}")
                    if lankle_clamped: clamped_joints.append(f"L_Ankle:{lankle}->{lankle_val:.1f}")
                    print(f"  ⚠️  Frame {frame_idx} clamped: {', '.join(clamped_joints)}")
            
            # Apply smoothing if enabled
            current_angles = [rhip_val, rknee_val, rankle_val, lhip_val, lknee_val, lankle_val]
            
            if args.smooth and prev_angles is not None:
                smoothed_angles = []
                for prev, curr in zip(prev_angles, current_angles):
                    smoothed = smooth_transition(prev, curr, args.smooth_factor)
                    smoothed_angles.append(smoothed)
                current_angles = smoothed_angles
            
            prev_angles = current_angles
            
            # Unpack smoothed/clamped angles
            rhip_final, rknee_final, rankle_final, lhip_final, lknee_final, lankle_final = current_angles
            
            # Build command
            cmd = build_angles_command(rhip_final, rknee_final, rankle_final, lhip_final, lknee_final, lankle_final)
            
            if args.dry_run or args.verbose:
                print(f"Frame {frame_idx}: {cmd.strip()}")
            
            # Send to Arduino and wait for completion
            if not args.dry_run:
                frame_start_time = time.time()
                
                success, response, completion_time = send_command_and_wait_completion(ser, cmd, timeout=10.0)
                
                if not success:
                    error_count += 1
                    print(f"✗ Frame {frame_idx} ERROR: {response}")
                    print(f"  Command: {cmd.strip()}")
                    
                    # Ask user what to do
                    user_input = input("  Continue (c), retry (r), or quit (q)? ").lower()
                    if user_input == 'q':
                        break
                    elif user_input == 'r':
                        # Retry once
                        success, response, completion_time = send_command_and_wait_completion(ser, cmd, timeout=10.0, max_retries=1)
                        if not success:
                            print(f"  Retry failed: {response}")
                
                # Enforce minimum frame delay (wait additional time if needed)
                elapsed_frame_time = time.time() - frame_start_time
                if elapsed_frame_time < min_frame_delay:
                    additional_delay = min_frame_delay - elapsed_frame_time
                    time.sleep(additional_delay)
                    total_frame_time = min_frame_delay
                else:
                    total_frame_time = elapsed_frame_time
                
                # Progress indicator with completion time
                if sent_count % 10 == 0 and sent_count > 0:
                    elapsed = time.time() - start_time
                    fps_actual = sent_count / elapsed
                    print(f"✓ Sent {sent_count} frames | {fps_actual:.2f} FPS actual | {clamp_count} clamped | Frame: {total_frame_time*1000:.0f}ms (move:{completion_time*1000:.0f}ms)")
                elif args.verbose and success:
                    print(f"  Frame {frame_idx} total: {total_frame_time*1000:.0f}ms (move:{completion_time*1000:.0f}ms + delay:{(total_frame_time-completion_time)*1000:.0f}ms)")
            
            sent_count += 1
    
    except KeyboardInterrupt:
        print("\n\n✋ Interrupted by user")
    
    finally:
        # Cleanup
        if ser:
            # Send STOP command
            print("\nStopping servos...")
            ser.write(b"STOP\n")
            time.sleep(0.1)
            ser.close()
        
        csv_file.close()
    
    # Summary
    elapsed = time.time() - start_time
    print("\n" + "="*60)
    print("SUMMARY (SYNCHRONOUS EXECUTION)")
    print("="*60)
    print(f"Frames sent:      {sent_count}")
    if skip_count > 0:
        print(f"Frames skipped:   {skip_count}")
    if clamp_count > 0:
        print(f"Frames clamped:   {clamp_count} ({clamp_count/max(sent_count,1)*100:.1f}%)")
    if error_count > 0:
        print(f"Errors:           {error_count}")
    print(f"Time elapsed:     {elapsed:.1f}s")
    if sent_count > 0:
        print(f"Actual FPS:       {sent_count/elapsed:.2f}")
        print(f"Avg time/frame:   {elapsed/sent_count*1000:.0f}ms")
    print(f"Execution mode:   Row-by-row (waits for Arduino COMPLETE)")
    print(f"Angle range:      {args.angle_min}° - {args.angle_max}°")
    print("="*60)


if __name__ == '__main__':
    main()