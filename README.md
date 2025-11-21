# Bionic Leg Control System - Complete Documentation

## Project Overview

This project implements a complete pipeline for capturing human gait using computer vision and replicating it on a 6-DOF bionic leg robot. The system consists of three main components:

1. **MediaPose Angle Capture** - Real-time joint angle detection using OpenCV
2. **Arduino Servo Controller** - Hardware control for 6 servo motors via PCA9685
3. **CSV-to-Arduino Streamer** - Bridge between captured gait data and hardware

---

## System Architecture

```
┌─────────────────────┐
│   MediaPose CV      │  Captures joint angles from camera/video
│  (Python + OpenCV)  │  
└──────────┬──────────┘
           │
           ▼
    output_angles.csv     Stores 6 joint angles per frame
           │
           ▼
┌─────────────────────┐
│  CSV Streamer       │  Filters, interpolates, sends angles
│  (pose_to_arduino)  │  
└──────────┬──────────┘
           │
           ▼ Serial (115200 baud)
┌─────────────────────┐
│  Arduino + PCA9685  │  Controls 6 servo motors
│  (Servo Controller) │  
└─────────────────────┘
```

---

## Hardware Requirements

### Electronics
- **Arduino Board** (Uno/Mega/Nano)
- **PCA9685** 16-channel PWM servo driver (I2C address 0x40)
- **6x Servo Motors** (MG996R or similar)
- **5V Power Supply** (adequate amperage for servos)

### Mechanical
- Bionic leg frame with 6 degrees of freedom:
  - Left Hip, Knee, Ankle
  - Right Hip, Knee, Ankle

### Computer
- Python 3.7+
- Webcam (for real-time capture) or video file

---

## Installation

### 1. Clone Repository
```bash
cd bionic-leg-mediapose-pipeline
```

### 2. Install Python Dependencies
```bash
pip install -r requirements_simulator.txt
pip install mediapipe opencv-python numpy pandas matplotlib scipy pyserial
```

### 3. Install Arduino Libraries
Open Arduino IDE and install:
- `Adafruit_PWMServoDriver` (for PCA9685)
- `Wire` (I2C communication, built-in)

---

## Part 1: MediaPose Angle Capture

### Purpose
Captures joint angles (hip, knee, ankle) from video input using MediaPipe pose detection and saves to CSV.

### Files
- `leg_angle_capture.py` - Basic angle capture
- `leg_angle_capture_real_time.py` - Real-time with UDP streaming
- `trial_1.py` - Advanced with smoothing and JSON output

### How It Works
1. **MediaPipe Detection**: Identifies 33 body landmarks in each frame
2. **Angle Calculation**: Computes 3D angles using vector geometry
   - Hip: Angle between shoulder-hip-knee
   - Knee: Angle between hip-knee-ankle
   - Ankle: Angle between knee-ankle-foot
3. **CSV Logging**: Saves angles to `output_angles.csv`

### CSV Format
```csv
timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle
2025-10-07 17:59:35.950,1,173.75,166.64,170.67,170.33,168.97,170.68
```

### Usage

#### Option A: Capture from Webcam
```bash
python leg_angle_capture_real_time.py --camera 0
```

#### Option B: Process Video File
```bash
python leg_angle_capture.py --video gait_video.mp4
```

#### Option C: With UDP Streaming (Advanced)
```bash
python leg_angle_capture_real_time.py --camera 0 --udp-ip 127.0.0.1 --udp-port 5005
```

### Output
- **CSV File**: `output_angles.csv` - Complete angle log
- **Video Window**: Real-time visualization with overlaid angles
- **Console**: Frame-by-frame angle readout

### Visualization Tools
After capture, visualize the data:

```bash
# Plot angles over time
python leg_angle_plotting.py --csv output_angles.csv

# With smoothing
python leg_plot_smoothened.py --csv output_angles.csv --smooth --method savgol
```

---

## Part 2: Arduino Servo Controller

### Purpose
Controls 6 servo motors via PCA9685 based on received angle commands from serial port.

### Files
- `arduino/sketch_nov16b_final/sketch_nov16b_final.ino` - Automated CSV playback controller
- `arduino/final_manual/sketch_final_manual.ino` - **[CURRENT WORKING VERSION]** Manual keyboard control

### Motor Mapping
| Joint | Motor | PCA9685 Channel | CSV Column |
|-------|-------|-----------------|------------|
| Left Ankle | M1 | 0 | `left_ankle_angle` |
| Right Ankle | M2 | 1 | `right_ankle_angle` |
| Left Knee | M3 | 2 | `left_knee_angle` |
| Right Knee | M4 | 3 | `right_knee_angle` |
| Left Hip | M5 | 4 | `left_hip_angle` |
| Right Hip | M6 | 5 | `right_hip_angle` |

### How It Works

#### 1. Automated Controller (`sketch_nov16b_final`)
- Receives `ANGLES,<r_hip>,<r_knee>,<r_ankle>,<l_hip>,<l_knee>,<l_ankle>` via serial
- Applies EMA (Exponential Moving Average) smoothing
- Maps angles to PWM values (150-600 range)
- Ramps PWM changes to prevent servo jerk
- Includes watchdog timeout (500ms)

**Protocol:**
```cpp
// Command format
ANGLES,170.5,165.3,172.1,168.9,169.2,171.0

// Emergency commands
STOP    // Centers all servos to 90°
CENTER  // Same as STOP
STATUS  // Reports current angles and PWM values
```

#### 2. Manual Controller (`sketch_final_manual`) [CURRENT WORKING VERSION]

This version provides direct keyboard control via Serial Monitor for testing and debugging.

**Why This Works:**
- No BNO055 dependency - Pure manual control without IMU sensors
- No 360° servo issues - Works with standard 180° servos
- Immediate response - No CSV parsing overhead
- Easy debugging - Direct visual feedback of motor positions

**Commands:**
```
rr - Right leg forward  (hip forward, knee bend)
rl - Right leg backward (hip back, knee straighten)
lr - Left leg forward
ll - Left leg backward

Format: Send 2-character string (e.g., "rr") via Serial Monitor
```

**How It Works:**
1. Waits for 2-character command on serial port
2. Directly sets PWM values for targeted motors
3. Smooth transitions using step-by-step PWM ramping
4. No angle calculations - works with raw PWM values

**Motor Positions (Example):**
```cpp
// Right Forward (rr)
M2 (Right Ankle) → 325 PWM   // Lift foot
M4 (Right Knee)  → 280 PWM   // Bend knee
M6 (Right Hip)   → 450 PWM   // Hip forward

// Right Backward (rl)
M2 → 291 PWM   // Lower foot
M4 → 500 PWM   // Straighten knee
M6 → 300 PWM   // Hip back
```

**Limitations & Future Work:**
- **No Closed-Loop Control**: Cannot use BNO055 IMU for feedback (not yet implemented)
- **No 360° Servo Support**: System designed for standard 180° range servos
- **Manual Tuning Required**: PWM values hardcoded, needs adjustment per hardware setup
- **Future Enhancement**: Implement BNO055 integration for angle feedback and self-correction

### Setup & Upload

#### 1. Arduino IDE Configuration
1. Open Arduino IDE
2. Select **Tools → Board** → Your Arduino model
3. Select **Tools → Port** → Your COM port (e.g., COM3)

#### 2. Wiring (PCA9685 to Arduino)
```
PCA9685 → Arduino
VCC     → 5V
GND     → GND
SDA     → A4 (Uno) / SDA pin (Mega)
SCL     → A5 (Uno) / SCL pin (Mega)
V+      → External 5V supply (for servos)
```

#### 3. Upload Code

**For Manual Control (Recommended for Testing):**
```bash
# Open sketch_final_manual.ino in Arduino IDE
File → Open → arduino/final_manual/sketch_final_manual.ino

# Upload to board
Sketch → Upload (or Ctrl+U)
```

**For Automated CSV Playback:**
```bash
# Open sketch_nov16b_final.ino in Arduino IDE
File → Open → arduino/sketch_nov16b_final/sketch_nov16b_final.ino

# Upload to board
Sketch → Upload
```

#### 4. Test Connection
```bash
# Open Serial Monitor (Tools → Serial Monitor)
# Set baud rate to 115200

# You should see:
PCA9685 initialized on channel 0
System ready

# Test with manual commands (sketch_final_manual):
rr   # Right leg forward
rl   # Right leg back
lr   # Left leg forward
ll   # Left leg back
```

---

## Part 3: CSV-to-Arduino Streamer

### Purpose
Bridges captured gait data (CSV) and Arduino hardware by streaming filtered, interpolated angles via serial port.

### Files
- `pose_to_arduino.py` - Advanced streamer with filtering and interpolation
- `send_csv_to_arduino.py` - Synchronous slow-speed version

### How It Works

#### Data Processing Pipeline
1. **CSV Reading**: Loads `output_angles.csv`
2. **Frame Interpolation**: Resamples to consistent frame rate (e.g., 50 Hz)
3. **Low-Pass Filtering**: Applies exponential moving average for smooth transitions
4. **Serial Transmission**: Sends `ANGLES` commands to Arduino
5. **Timing Control**: Maintains precise playback rate

#### Low-Pass Filter (EMA)
```python
# Exponential Moving Average
filtered_angle = alpha * new_angle + (1 - alpha) * prev_angle
# alpha = 0.2 (default) - Lower = smoother, Higher = more responsive
```

#### Frame Interpolation
```python
# If CSV has 30 FPS but playback is 50 FPS:
# System interpolates intermediate frames for smooth motion
interpolated_angle = angle_frame1 + (angle_frame2 - angle_frame1) * fraction
```

### Usage

#### Basic Playback
```bash
# List available serial ports
python pose_to_arduino.py --list-ports

# Dry run (no hardware, just print commands)
python pose_to_arduino.py --csv output_angles.csv --dry-run

# Send to Arduino at 50 Hz
python pose_to_arduino.py --port COM3 --csv output_angles.csv --rate 50
```

#### Advanced Options
```bash
# With low-pass filtering (smoother motion)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --rate 50 --filter-alpha 0.3

# Loop continuously
python pose_to_arduino.py --port COM3 --csv output_angles.csv --loop

# Verbose output (see every command)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --verbose

# Relative movement mode (uses deltas from baseline)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --baseline-frame 1
```

#### Synchronous Slow Version (Safe Testing)
```bash
# Very slow playback (500ms per frame) - waits for Arduino completion
python send_csv_to_arduino.py --port COM3 --csv output_angles.csv

# Custom delay per frame
python send_csv_to_arduino.py --port COM3 --min-delay 1000  # 1 second/frame

# With angle smoothing
python send_csv_to_arduino.py --port COM3 --smooth --smooth-factor 0.3

# Skip frames with missing data
python send_csv_to_arduino.py --port COM3 --skip-empty
```

### Command-Line Arguments

#### `pose_to_arduino.py`
| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | - | Serial port (e.g., COM3, /dev/ttyUSB0) |
| `--csv` | `output_angles.csv` | Input CSV file |
| `--rate` | 50 | Playback rate in Hz |
| `--baud` | 115200 | Serial baud rate |
| `--filter-alpha` | 0.2 | Low-pass filter strength (0-1) |
| `--relative` | False | Enable relative movement mode |
| `--baseline-frame` | 1 | Reference frame for relative mode |
| `--loop` | False | Loop playback continuously |
| `--dry-run` | False | Print commands without sending |
| `--verbose` | False | Detailed output |

#### `send_csv_to_arduino.py`
| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | - | Serial port |
| `--csv` | `output_angles.csv` | Input CSV file |
| `--fps` | 2 | Frames per second |
| `--min-delay` | - | Minimum ms per frame (overrides FPS) |
| `--smooth` | False | Enable angle smoothing |
| `--smooth-factor` | 0.3 | Smoothing strength (0.1-1.0) |
| `--angle-min` | 60 | Minimum safe angle |
| `--angle-max` | 120 | Maximum safe angle |
| `--dry-run` | False | Test without hardware |

### Safety Features
- **Angle Clamping**: Prevents servo over-rotation (60°-120° by default)
- **Smooth Transitions**: Exponential filtering reduces jerky movements
- **Watchdog Timeout**: Arduino stops if no commands for 500ms
- **Emergency Stop**: Ctrl+C gracefully stops and centers servos

---

## Complete Workflow Examples

### Example 1: Capture → Visualize → Test
```bash
# 1. Capture gait from webcam
python leg_angle_capture_real_time.py --camera 0
# Press 'q' to stop after capturing ~5 seconds

# 2. Visualize captured data
python leg_plot_smoothened.py --csv output_angles.csv --smooth

# 3. Test with simulator (no hardware needed)
python simulate_robot_leg.py --csv output_angles.csv --fps 30

# 4. Dry-run to Arduino (verify commands)
python pose_to_arduino.py --csv output_angles.csv --dry-run
```

### Example 2: Manual Robot Control (Current Working Method)
```bash
# 1. Upload manual control sketch
# Open arduino/final_manual/sketch_final_manual.ino
# Arduino IDE → Upload

# 2. Open Serial Monitor
# Tools → Serial Monitor
# Set baud: 115200

# 3. Type commands directly:
rr   # Right leg forward
rl   # Right leg backward
lr   # Left leg forward
ll   # Left leg backward

# Commands are executed immediately
```

### Example 3: Automated Playback (Safe Testing)
```bash
# 1. Upload automated sketch
# Open arduino/sketch_nov16b_final/sketch_nov16b_final.ino
# Arduino IDE → Upload

# 2. Start with VERY slow playback
python send_csv_to_arduino.py --port COM3 --min-delay 2000
# 2 seconds per frame - allows visual inspection

# 3. Increase speed gradually
python send_csv_to_arduino.py --port COM3 --min-delay 1000  # 1s/frame
python send_csv_to_arduino.py --port COM3 --min-delay 500   # 0.5s/frame

# 4. Full speed with filtering
python pose_to_arduino.py --port COM3 --rate 30 --filter-alpha 0.3
```

### Example 4: Production Playback
```bash
# 1. Ensure robot is securely mounted/hanging

# 2. Start playback with safety features
python pose_to_arduino.py \
  --port COM3 \
  --csv output_angles.csv \
  --rate 50 \
  --filter-alpha 0.2 \
  --loop \
  --verbose

# Press Ctrl+C to stop (will center servos automatically)
```

---

## Visualization & Analysis Tools

### Simulator
Visual preview of gait without hardware:
```bash
# Launch interactive simulator
python simulate_robot_leg.py --csv output_angles.csv

# Controls:
# SPACE - Pause/resume
# LEFT/RIGHT - Step through frames
# UP/DOWN - Adjust speed
# G - Toggle angle graph
# R - Reset to beginning
# Q/ESC - Quit
```

See `README_SIMULATOR.md` for full simulator documentation.

### Plotting Tools
```bash
# Basic angle plots
python leg_angle_plotting.py --csv output_angles.csv

# With Savitzky-Golay smoothing
python leg_plot_smoothened.py --csv output_angles.csv --smooth --method savgol

# With moving average
python leg_plot_smoothened.py --csv output_angles.csv --smooth --method ma
```

---

## CSV Data Format

### Structure
```csv
timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle
2025-10-07 17:59:35.950,1,173.75,166.64,170.67,170.33,168.97,170.68
2025-10-07 17:59:36.110,2,175.38,169.67,172.09,171.79,169.68,170.54
```

### Column Descriptions
- `timestamp`: Capture time (YYYY-MM-DD HH:MM:SS.mmm)
- `frame`: Sequential frame number
- `right_hip_angle`: Right hip angle in degrees (0°-180°)
- `right_knee_angle`: Right knee angle in degrees
- `right_ankle_angle`: Right ankle angle in degrees
- `left_hip_angle`: Left hip angle in degrees
- `left_knee_angle`: Left knee angle in degrees
- `left_ankle_angle`: Left ankle angle in degrees

### Angle Convention
- **0°**: Fully extended (leg straight)
- **90°**: Neutral standing position (reference in code: `180 - detected_angle`)
- **180°**: Fully flexed (maximum bend)

---

## Calibration & Tuning

### Servo Calibration
Each servo has unique PWM limits. Document your hardware in `calib.json`:

```json
{
  "servos": {
    "left_ankle": {
      "channel": 0,
      "min_pwm": 292,
      "max_pwm": 324,
      "min_angle": 60,
      "max_angle": 120,
      "inverted": false
    }
  }
}
```

### Finding PWM Limits
```bash
# 1. Center all servos
# Arduino Serial Monitor → Send: CENTER

# 2. Test one servo at a time
# Slowly increase PWM until joint reaches physical limit
# Example for Left Ankle:
ANGLES,90,90,90,90,90,60    # Test minimum
ANGLES,90,90,90,90,90,120   # Test maximum

# 3. Record values in calib.json
```

### Filter Tuning
```python
# Low-pass filter alpha (0-1)
# Lower = smoother but more lag
# Higher = more responsive but less smooth

# For slow walking:
--filter-alpha 0.1

# For fast movements:
--filter-alpha 0.3

# No filtering:
--filter-alpha 1.0
```

---

## Troubleshooting

### MediaPose Issues

**Problem:** No pose detected
```bash
# Solution: Ensure good lighting and full body visibility
# Check camera is working:
python -c "import cv2; print(cv2.__version__)"
```

**Problem:** Angles look incorrect
```bash
# Solution: Check angle convention (180 - detected_angle)
# Visualize to verify:
python leg_plot_smoothened.py --csv output_angles.csv --smooth
```

### Arduino Connection Issues

**Problem:** Port not found
```bash
# List available ports:
python pose_to_arduino.py --list-ports

# Windows: Usually COM3, COM4, etc.
# Linux/Mac: Usually /dev/ttyUSB0, /dev/ttyACM0
```

**Problem:** Arduino not responding
```bash
# 1. Check Serial Monitor (115200 baud)
# 2. Verify upload was successful
# 3. Press Arduino reset button
# 4. Try different USB cable/port
```

**Problem:** Servos not moving
```bash
# Check wiring:
# - PCA9685 I2C address (default 0x40)
# - Servo power supply (adequate current)
# - Servo signal wires on correct channels

# Test with manual commands:
# Serial Monitor → Send: CENTER
# All servos should move to neutral position
```

### Playback Issues

**Problem:** Jerky motion
```bash
# Increase filtering:
python pose_to_arduino.py --port COM3 --filter-alpha 0.1

# Decrease playback speed:
python pose_to_arduino.py --port COM3 --rate 30
```

**Problem:** Robot moves too fast
```bash
# Use synchronous mode:
python send_csv_to_arduino.py --port COM3 --min-delay 1000

# Or reduce playback rate:
python pose_to_arduino.py --port COM3 --rate 10
```

**Problem:** Serial timeout
```bash
# Increase timeout in code or:
# Use lower baud rate:
python pose_to_arduino.py --port COM3 --baud 9600
```

---

## Project Structure

```
bionic-leg-mediapose-pipeline/
├── arduino/
│   ├── sketch_nov16b_final/          # Automated CSV controller
│   └── final_manual/                 # Manual keyboard control [WORKING]
│
├── leg_angle_capture.py              # Basic MediaPose capture
├── leg_angle_capture_real_time.py    # Real-time with UDP
├── trial_1.py                        # Advanced capture with smoothing
│
├── pose_to_arduino.py                # Main CSV streamer
├── send_csv_to_arduino.py            # Synchronous slow streamer
│
├── simulate_robot_leg.py             # Pygame simulator
├── simulate_robot_leg_matplotlib.py  # Matplotlib alternative
│
├── leg_angle_plotting.py             # Basic angle plots
├── leg_plot_smoothened.py            # Smoothed angle plots
├── leg_angles_visualiazer.py         # Alternative visualizer
│
├── output_angles.csv                 # Captured gait data
├── calib.json                        # Servo calibration
│
├── README_SIMULATOR.md               # Simulator documentation
├── README_RELATIVE_MOVEMENT.md       # Relative mode guide
├── full_specifications.md            # Full system specification
├── SIMULATION.md                     # Simulation details
└── requirements_simulator.txt        # Python dependencies
```

---

## Quick Start Guide

### First-Time Setup (30 minutes)

#### 1. Software Installation (10 min)
```bash
# Install Python packages
pip install mediapipe opencv-python numpy pandas matplotlib scipy pyserial pygame

# Install Arduino libraries
# Arduino IDE → Library Manager → Search "Adafruit PWM Servo"
```

#### 2. Hardware Assembly (10 min)
```bash
# Connect PCA9685 to Arduino (I2C)
# Connect servos to PCA9685 (channels 0-5)
# Connect external 5V power to PCA9685 V+
```

#### 3. Upload Arduino Code (5 min)
```bash
# Open arduino/final_manual/sketch_final_manual.ino
# Arduino IDE → Upload
# Serial Monitor → Check "System ready" message
```

#### 4. Test Manual Control (5 min)
```bash
# Serial Monitor → Send commands:
rr   # Right forward
rl   # Right back
lr   # Left forward
ll   # Left back

# Verify servos respond correctly
```

### Daily Operation (5 minutes)

#### Option A: Manual Control
```bash
# 1. Open Serial Monitor (115200 baud)
# 2. Send 2-character commands (rr, rl, lr, ll)
# 3. Observe robot movement
```

#### Option B: Automated Playback
```bash
# 1. Capture gait
python leg_angle_capture_real_time.py --camera 0

# 2. Preview in simulator
python simulate_robot_leg.py --csv output_angles.csv

# 3. Send to robot (slow and safe)
python send_csv_to_arduino.py --port COM3 --min-delay 1000

# 4. Full speed
python pose_to_arduino.py --port COM3 --rate 50
```

---

## Additional Documentation

- **`full_specifications.md`** - Complete system specification
- **`README_SIMULATOR.md`** - Pygame simulator guide
- **`README_RELATIVE_MOVEMENT.md`** - Relative movement mode
- **`SIMULATION.md`** - Simulation technical details
- **`VISUALIZATION_UPGRADE.md`** - Visual improvements guide

---

## Contributing

This project is part of bionic leg gait replication research. For issues or improvements, please document:
1. Hardware configuration
2. Calibration values
3. CSV data samples
4. Observed behavior vs. expected behavior

---

## Safety Warning

- Always test with robot suspended before floor operation
- Never exceed servo mechanical limits
- Use emergency stop (Ctrl+C) if unexpected behavior
- Calibrate PWM limits before full-speed operation
- Secure power supply with adequate amperage

---

## Future Enhancements

### Planned Features
- BNO055 IMU Integration - Closed-loop angle feedback
- 360° Servo Support - Continuous rotation joints
- Wireless Control - ESP32 with WiFi/Bluetooth
- Mobile App - Remote monitoring and control
- Gait Library - Pre-recorded movement patterns
- Auto-Calibration - Servo limit detection
- Multi-Robot Sync - Coordinated movement

### Current Limitations
- No closed-loop control (BNO055 not implemented)
- Limited to 180° servos (no continuous rotation)
- Manual calibration required per hardware setup
- Serial communication only (no wireless)

---

## Support

For questions or issues:
1. Check `full_specifications.md` for detailed specifications
2. Review troubleshooting section above
3. Verify hardware connections and calibration
4. Test with `simulate_robot_leg.py` first

---

**Built for bionic leg gait replication research**

**License:** Part of bionic-leg-mediapose-pipeline project

**Last Updated:** November 22, 2025
