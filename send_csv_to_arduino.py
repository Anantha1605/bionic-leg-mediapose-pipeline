#!/usr/bin/env python3
"""
send_csv_to_arduino.py

Reads `output_angles.csv` from the repo and streams hip/knee/ankle angles to an
Arduino over serial. By default uses the right leg columns; you can choose left.

Usage:
  python send_csv_to_arduino.py --port COM3
  python send_csv_to_arduino.py --port /dev/ttyUSB0 --fps 30
  python send_csv_to_arduino.py --dry-run  # Test without hardware

Options:
  --port COMx   Serial port (required unless --dry-run)
  --baud N      Baud rate (default 115200)
  --side R|L    R for right leg (default), L for left leg
  --delay ms    Delay between rows in milliseconds (default 40)
  --dry-run     Parse CSV and print commands without sending to serial
  --skip-empty  Skip rows with missing angles

The script sends lines in format: <Side>,<hip>,<knee>,<ankle>\n and waits for an "OK" reply.
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


def find_ports():
    """List available serial ports"""
    if serial is None:
        return []
    return [p.device for p in list_ports.comports()]


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


def send_command_with_retry(ser, cmd, max_retries=3):
    """
    Send command and wait for OK response with retry logic
    Returns: (success, response_text)
    """
    for attempt in range(max_retries):
        try:
            ser.write(cmd.encode('utf-8'))
            ser.flush()
            
            # Wait for response (timeout from serial object)
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if response == "OK":
                return True, response
            elif response.startswith("ERR_"):
                return False, response
            else:
                # Unexpected response
                if attempt < max_retries - 1:
                    print(f"  Retry {attempt + 1}/{max_retries}: got '{response}'")
                    time.sleep(0.05)
                else:
                    return False, response
        
        except serial.SerialException as e:
            print(f"  Serial error: {e}")
            if attempt < max_retries - 1:
                time.sleep(0.1)
            else:
                return False, str(e)
    
    return False, "Max retries exceeded"


def main():
    parser = argparse.ArgumentParser(
        description='Stream CSV joint angles to Arduino servo controller',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--port', help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--fps', type=float, default=30.0, help='Playback FPS (default: 30)')
    parser.add_argument('--csv', default=CSV_PATH, help=f'CSV file path (default: {CSV_PATH})')
    parser.add_argument('--dry-run', action='store_true', help='Print commands without sending')
    parser.add_argument('--skip-empty', action='store_true', help='Skip rows with missing data')
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
    frame_delay = 1.0 / args.fps
    print(f"Playback rate: {args.fps} FPS ({frame_delay*1000:.1f}ms per frame)\n")
    
    if args.dry_run:
        print("=== DRY RUN MODE (not sending to Arduino) ===\n")
    
    # Process CSV rows
    sent_count = 0
    error_count = 0
    skip_count = 0
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
            rhip = rhip or '90'
            rknee = rknee or '90'
            rankle = rankle or '90'
            lhip = lhip or '90'
            lknee = lknee or '90'
            lankle = lankle or '90'
            
            # Build command
            cmd = build_angles_command(rhip, rknee, rankle, lhip, lknee, lankle)
            
            if args.dry_run or args.verbose:
                print(f"Frame {frame_idx}: {cmd.strip()}")
            
            # Send to Arduino
            if not args.dry_run:
                success, response = send_command_with_retry(ser, cmd)
                
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
                        success, response = send_command_with_retry(ser, cmd, max_retries=1)
                        if not success:
                            print(f"  Retry failed: {response}")
                
                # Progress indicator
                if sent_count % 30 == 0 and sent_count > 0:
                    elapsed = time.time() - start_time
                    fps_actual = sent_count / elapsed
                    print(f"✓ Sent {sent_count} frames ({fps_actual:.1f} FPS)")
            
            sent_count += 1
            
            # Frame timing
            if not args.dry_run:
                time.sleep(frame_delay)
    
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
    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    print(f"Frames sent:    {sent_count}")
    if skip_count > 0:
        print(f"Frames skipped: {skip_count}")
    if error_count > 0:
        print(f"Errors:         {error_count}")
    print(f"Time elapsed:   {elapsed:.1f}s")
    if sent_count > 0:
        print(f"Actual FPS:     {sent_count/elapsed:.1f}")
    print("="*50)


if __name__ == '__main__':
    main()