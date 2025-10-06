#!/usr/bin/env python3
"""
send_csv_to_arduino.py

Reads `output_angles.csv` from the repo and streams hip/knee/ankle angles to an
Arduino over serial. By default uses the right leg columns; you can choose left.

Usage:
  python send_csv_to_arduino.py --port COM3 --baud 115200 --side R

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
except Exception:
    serial = None

CSV_PATH = 'output_angles.csv'


def find_ports():
    if serial is None:
        return []
    return [p.device for p in list_ports.comports()]


def build_command(side, hip, knee, ankle):
    # Ensure numeric values, limit to 0-360 inputs (we handle >180 in Arduino)
    def fmt(v):
        return str(int(round(float(v))))

    return f"{side},{fmt(hip)},{fmt(knee)},{fmt(ankle)}\n"


def open_serial(port, baud):
    if serial is None:
        raise RuntimeError('pyserial not installed')
    ser = serial.Serial(port, baud, timeout=2)
    # small delay to allow Arduino to reset if auto-reset is enabled
    time.sleep(2.0)
    ser.flushInput()
    return ser


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', required=False)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--side', choices=['R', 'L', 'B'], default='R',
                   help='R or L to stream one leg, B to stream both legs from CSV (recommended)')
    p.add_argument('--delay', type=float, default=40.0)
    p.add_argument('--dry-run', action='store_true')
    p.add_argument('--skip-empty', action='store_true')
    p.add_argument('--cal-file', type=str, default=None,
                   help='JSON or CSV file with calibration offsets and invert flags')
    p.add_argument('--apply-cal', action='store_true',
                   help='If set, upload calibration to Arduino before streaming')
    args = p.parse_args()

    if args.port is None and not args.dry_run:
        print('No --port supplied and not --dry-run. Available ports:')
        for port in find_ports():
            print('  ', port)
        sys.exit(1)

    # Open CSV
    try:
        f = open(CSV_PATH, newline='')
    except FileNotFoundError:
        print(f'CSV file not found at {CSV_PATH}')
        sys.exit(1)

    reader = csv.DictReader(f)

    # Determine column names
    if args.side == 'R':
        hip_col = 'right_hip_angle'
        knee_col = 'right_knee_angle'
        ankle_col = 'right_ankle_angle'
    elif args.side == 'L':
        hip_col = 'left_hip_angle'
        knee_col = 'left_knee_angle'
        ankle_col = 'left_ankle_angle'
    else:
        # B (both) - we will read right then left for one combined command
        hip_col_r = 'right_hip_angle'
        knee_col_r = 'right_knee_angle'
        ankle_col_r = 'right_ankle_angle'
        hip_col_l = 'left_hip_angle'
        knee_col_l = 'left_knee_angle'
        ankle_col_l = 'left_ankle_angle'

    ser = None
    if not args.dry_run:
        ser = open_serial(args.port, args.baud)

    # If calibration file provided and --apply-cal, parse and upload
    cal_entries = None
    if args.cal_file:
        import json
        try:
            with open(args.cal_file, 'r') as cf:
                if args.cal_file.lower().endswith('.json'):
                    cal_entries = json.load(cf)
                else:
                    # try to parse CSV: servo_idx,offset,invert
                    cal_entries = []
                    for line in cf:
                        parts = line.strip().split(',')
                        if len(parts) >= 3:
                            cal_entries.append({'idx': int(parts[0]), 'offset': float(parts[1]), 'invert': int(parts[2])})
        except Exception as e:
            print('Failed to parse calibration file:', e)
            cal_entries = None

    if ser and args.apply_cal and cal_entries:
        # send calibration entries
        for e in cal_entries:
            idx = e.get('idx')
            offset = e.get('offset')
            inv = e.get('invert', 0)
            cmd = f"CAL,{idx},{offset},{inv}\n"
            ser.write(cmd.encode('utf-8'))
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print('CAL RESP:', line)
        # save to EEPROM
        ser.write(b'SAVE_CAL\n')
        print('SAVE_CAL RESP:', ser.readline().decode('utf-8', errors='ignore').strip())

    sent = 0
    for row in reader:
        if args.side in ('R', 'L'):
            hip = row.get(hip_col, '').strip()
            knee = row.get(knee_col, '').strip()
            ankle = row.get(ankle_col, '').strip()

            if args.skip_empty and (hip == '' or knee == '' or ankle == ''):
                continue

            # Fallbacks
            if hip == '': hip = 90
            if knee == '': knee = 90
            if ankle == '': ankle = 90

            cmd = build_command(args.side, hip, knee, ankle)
        else:
            # Both legs combined
            rhip = row.get(hip_col_r, '').strip()
            rknee = row.get(knee_col_r, '').strip()
            rankle = row.get(ankle_col_r, '').strip()
            lhip = row.get(hip_col_l, '').strip()
            lknee = row.get(knee_col_l, '').strip()
            lankle = row.get(ankle_col_l, '').strip()

            if args.skip_empty and (rhip == '' or rknee == '' or rankle == '' or lhip == '' or lknee == '' or lankle == ''):
                continue

            # Fallbacks
            rhip = rhip if rhip != '' else 90
            rknee = rknee if rknee != '' else 90
            rankle = rankle if rankle != '' else 90
            lhip = lhip if lhip != '' else 90
            lknee = lknee if lknee != '' else 90
            lankle = lankle if lankle != '' else 90

            cmd = f"B,{int(round(float(rhip)))},{int(round(float(rknee)))},{int(round(float(rankle)))},{int(round(float(lhip)))},{int(round(float(lknee)))},{int(round(float(lankle)))}\n"

        if args.dry_run:
            print(cmd.strip())
        else:
            ser.write(cmd.encode('utf-8'))
            # wait for OK
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line != 'OK':
                print(f'Warning: unexpected reply: "{line}"')
            sent += 1
            time.sleep(args.delay / 1000.0)

    if ser:
        ser.close()

    print(f'Done. Sent {sent} commands.')


if __name__ == '__main__':
    main()
