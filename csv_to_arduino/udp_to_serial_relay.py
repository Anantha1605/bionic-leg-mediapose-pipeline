# udp_to_serial_relay.py
# listens on UDP and writes incoming angle lines to Arduino serial
# windows usage: python udp_to_serial_relay.py
import socket, serial, time, threading, sys

# ===== USER CONFIG =====
SERIAL_PORT = "COM3"         # change to your port (Device Manager)
SERIAL_BAUD = 115200         # must match Arduino
UDP_IP = "0.0.0.0"           # listen on all local interfaces
UDP_PORT = 5005
WATCHDOG_MS = 300            # if no frame for this many ms -> send STOP
PRINT_INCOMING = False       # set True to debug
# =======================

last_frame_time = 0
running = True

def serial_thread_fn(port,baud):
    global last_frame_time, running
    try:
        ser = serial.Serial(port, baud, timeout=0.01)
    except Exception as e:
        print("failed to open serial port",port,"->",e)
        running=False
        return

    print(f"serial opened {port}@{baud}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.1)

    # small helper: write line to arduino (encoded)
    def write_line(line):
        if not line.endswith("\n"): line = line + "\n"
        try:
            ser.write(line.encode('utf-8'))
        except Exception as e:
            print("serial write error:",e)

    print(f"listening UDP {UDP_IP}:{UDP_PORT} -> forwarding to serial")
    write_line("PRINT\n") # optional initial debug
    while running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            # watchdog check below
            data = None
        if data:
            s = data.decode('utf-8',errors='ignore').strip()
            if not s: continue
            last_frame_time = int(time.time() * 1000)
            # optionally print or filter
            if PRINT_INCOMING:
                print("udp recv:", s[:120])
            # basic validation: expect at least 7 comma fields (frame + 6 angles)
            parts = s.replace("\t",",").split(",")
            numeric_count = sum(1 for p in parts if any(ch.isdigit() for ch in p))
            if numeric_count >= 7:
                # forward original line (Arduino parser is robust to timestamps)
                write_line(s)
            else:
                # ignore / optionally log
                if PRINT_INCOMING:
                    print("ignored incoming (not enough numeric fields)")
        # watchdog: if no frames recently -> send STOP once
        now_ms = int(time.time() * 1000)
        if last_frame_time != 0 and (now_ms - last_frame_time > WATCHDOG_MS):
            # send emergency stop command to arduino (single time until next frame)
            write_line("STOP\n")
            last_frame_time = now_ms  # avoid repeat spam until next frame
        # tiny sleep to avoid tight loop
        time.sleep(0.001)

    ser.close()
    sock.close()
    print("serial thread exiting")

def keyboard_thread():
    global running
    print("keyboard thread started: type 's' to STOP (center), 'q' to quit")
    while running:
        try:
            c = sys.stdin.readline().strip().lower()
            if not c: continue
            if c == "s":
                # emergency stop -> center
                print("-> sending STOP/CENTER")
                try:
                    ser_tmp = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
                    ser_tmp.write(b"STOP\n")
                    ser_tmp.write(b"CENTER\n")
                    ser_tmp.close()
                except Exception as e:
                    print("failed to open serial for emergency:", e)
            elif c == "q":
                print("quit requested")
                running = False
                break
            else:
                print("unknown key, q=quit, s=stop/center")
        except Exception:
            break

if __name__ == "__main__":
    t = threading.Thread(target=serial_thread_fn, args=(SERIAL_PORT,SERIAL_BAUD), daemon=True)
    t.start()
    # keyboard thread runs in main thread (so Ctrl+C works)
    try:
        keyboard_thread()
    except KeyboardInterrupt:
        running=False
    t.join()
    print("relay stopped")
