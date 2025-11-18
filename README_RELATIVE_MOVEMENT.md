# 🦿 Bionic Leg - CSV to Arduino Relative Movement Controller

Complete system for controlling a 6-DOF bionic leg using MediaPose angle data with **relative movement** for smooth, natural gait replication.

## 📋 Overview

This system reads joint angles from CSV files (captured from OpenCV MediaPose) and controls 6 servo motors via Arduino + PCA9685, implementing:

- ✅ **Relative Movement Mode**: Calculates deltas from baseline for natural gait
- ✅ **Low-pass Filtering**: Smooth angle transitions (exponential moving average)
- ✅ **Frame Interpolation**: Consistent timing at configurable frame rates
- ✅ **PWM Ramping**: Gradual servo movement to prevent jerk
- ✅ **Safety Features**: Hard limits, watchdog timeout, emergency stop
- ✅ **Calibration System**: Per-joint PWM limits and direction inversion

---

## 🔧 Hardware Requirements

### Electronics
- **Arduino Board**: Uno, Mega, Nano, or ESP32
- **PCA9685**: 16-channel 12-bit PWM servo driver (I2C address 0x40)
- **TCA9548A** (optional): I2C multiplexer for multiple PCA9685 boards
- **6x Servo Motors**: MG996, DS3218, or similar (tested with MG996)
- **Power Supply**: 5-6V, 5-10A capability (DO NOT power servos from Arduino)

### Mechanical
- Biped robot leg frame with 6 DOF (hip/knee/ankle × 2)
- Servo horns and linkages
- Proper mounting with minimal friction

---

## 🔌 Wiring Diagram

```
┌──────────────┐
│   Arduino    │
│   (5V/3.3V)  │
└──────┬───────┘
       │ I2C (SDA/SCL)
       │
┌──────▼───────────┐
│   TCA9548A       │  (Optional multiplexer)
│   I2C Mux        │
└──────┬───────────┘
       │ Channel 0
       │
┌──────▼───────────┐
│    PCA9685       │
│  Servo Driver    │
│  (0x40)          │
└──────┬───────────┘
       │ PWM Outputs
       │
       ├─ CH0 → M1 (Left Ankle)
       ├─ CH1 → M2 (Right Ankle)
       ├─ CH2 → M3 (Left Knee)
       ├─ CH3 → M4 (Right Knee)
       ├─ CH4 → M5 (Left Hip)
       └─ CH5 → M6 (Right Hip)

External 5-6V Power Supply (5-10A)
       ├─ (+) → PCA9685 V+ Terminal
       ├─ (-) → PCA9685 GND + Arduino GND (COMMON GROUND!)
       └─ Servo power pins
```

### Pin Connections

| Component | Arduino Pin | Notes |
|-----------|-------------|-------|
| SDA | A4 (Uno) / 20 (Mega) | I2C Data |
| SCL | A5 (Uno) / 21 (Mega) | I2C Clock |
| GND | GND | **Must share common ground with servo power** |

---

## 📦 Installation

### 1. Arduino Setup

**Required Libraries:**
```bash
# Install via Arduino Library Manager:
- Adafruit PWM Servo Driver Library
- Wire (built-in)
```

**Upload Sketch:**
1. Open `ArduinoRelativeMovement/ArduinoRelativeMovement.ino` in Arduino IDE
2. Select your board type and COM port
3. Upload the sketch
4. Open Serial Monitor at 115200 baud to verify startup

### 2. Python Setup

**Install Dependencies:**
```bash
pip install pyserial
```

**Files:**
- `pose_to_arduino.py` - Main CSV streamer with filtering
- `output_angles.csv` - Your captured gait data
- `calib.json` - Servo calibration configuration

---

## ⚙️ Calibration Process

**CRITICAL**: Calibrate before full-speed operation to prevent servo damage!

### Step 1: Mechanical Check
- [ ] Enlarge hip servo horn holes if needed
- [ ] Reduce knee hinge friction
- [ ] Ensure servos can freely move joints without binding

### Step 2: Find PWM Limits

For each servo motor, slowly sweep PWM to find physical limits:

```python
# Use Arduino Serial Monitor or Python script
# Start at center (PWM ~350 for most servos)
# Slowly increase/decrease until joint reaches physical stop

# Example for Left Ankle (Channel 0):
ANGLES,90,90,90,90,90,90     # Center all
ANGLES,90,90,90,90,90,0      # Move L_Ankle to min
ANGLES,90,90,90,90,90,180    # Move L_Ankle to max
```

**Record MIN/MAX PWM values where servo reaches stops (not stalling!)**

### Step 3: Update Calibration

Edit `calib.json` with measured values:

```json
{
  "joints": {
    "left_ankle": {
      "pwm_min": 292,    // ← Update with your measured MIN
      "pwm_max": 324,    // ← Update with your measured MAX
      "invert": false    // ← Set true if direction is reversed
    }
  }
}
```

### Step 4: Load to Arduino

Send calibration commands via Serial Monitor:

```
CAL,5,292,324,0    // Left Ankle: joint_id, min, max, invert
CAL,2,291,325,1    // Right Ankle (inverted)
CAL,4,309,324,1    // Left Knee (inverted)
SAVE_CAL           // Store in EEPROM
PRINT_CAL          // Verify
```

### Current Calibration Status

| Joint | Motor | Channel | MIN PWM | MAX PWM | Inverted | Calibrated |
|-------|-------|---------|---------|---------|----------|------------|
| L_Ankle | M1 | 0 | 292 | 324 | No | ✅ Yes |
| R_Ankle | M2 | 1 | 291 | 325 | Yes | ✅ Yes |
| L_Knee | M3 | 2 | 309 | 324 | Yes | ✅ Yes |
| R_Knee | M4 | 3 | 280 | 500 | Yes | ⚠️ Assumed |
| L_Hip | M5 | 4 | 300 | 450 | No | ❌ Pending |
| R_Hip | M6 | 5 | 300 | 450 | No | ❌ Pending |

---

## 🚀 Usage

### Basic CSV Playback

```bash
# Dry run (print commands without sending)
python pose_to_arduino.py --csv output_angles.csv --dry-run

# List available serial ports
python pose_to_arduino.py --list-ports

# Send to Arduino at 50Hz (absolute mode)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --rate 50

# Enable RELATIVE movement mode (recommended for gait)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --baseline-frame 1

# Loop continuously
python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --loop

# Verbose output
python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --verbose
```

### Advanced Options

```bash
# Adjust low-pass filter (higher = more smoothing)
python pose_to_arduino.py --port COM3 --csv output_angles.csv --filter-alpha 0.3

# Custom frame rate
python pose_to_arduino.py --port COM3 --csv output_angles.csv --rate 30

# Different baseline reference
python pose_to_arduino.py --port COM3 --csv output_angles.csv --relative --baseline-frame 10
```

### Arduino Serial Commands

Connect via Serial Monitor (115200 baud):

```
# Emergency stop (center all servos)
STOP

# Center all joints to 90°
CENTER

# Get status
STATUS

# Set angles manually (r_hip, r_knee, r_ankle, l_hip, l_knee, l_ankle)
ANGLES,90,45,120,90,45,120

# Set baseline reference
BASELINE,173,166,170,170,168,170

# Toggle relative mode
RELATIVE,1    // Enable
RELATIVE,0    // Disable

# Calibration
CAL,0,300,450,0         // joint_id, pwm_min, pwm_max, invert
SAVE_CAL                // Save to EEPROM
LOAD_CAL                // Load from EEPROM
PRINT_CAL               // Print current calibration
```

---

## 🎯 Movement Modes

### Absolute Mode (Default in older code)
Direct angle control - servo moves to exact angle from CSV.

**Pros**: Simple, predictable  
**Cons**: No smooth transitions between different capture sessions

### Relative Mode (Recommended ⭐)
Calculates delta from baseline reference, applies to current position.

**Pros**: Smooth, natural gait replication  
**Cons**: Requires baseline calibration

**How it works:**
```python
baseline = [173, 166, 170, 170, 168, 170]  # Frame 1 from CSV
current = [90, 90, 90, 90, 90, 90]         # Neutral starting position

csv_frame = [175, 169, 172, 171, 169, 170] # New frame from CSV
delta = csv_frame - baseline               # [2, 3, 2, 1, 1, 0]
target = current + delta                   # [92, 93, 92, 91, 91, 90]
```

---

## 🛡️ Safety Features

### Hardware Protection
- **Hard PWM Clamps**: Enforced from calibration MIN/MAX
- **Ramp Limiting**: Max 2 PWM units per 20ms cycle
- **External Power**: Servos powered separately from Arduino

### Software Protection
- **Watchdog Timer**: 500ms timeout → automatic STOP
- **NaN Handling**: Invalid angles default to 90° (neutral)
- **Angle Constraints**: 0-180° enforced
- **Emergency Stop**: `STOP` command or Ctrl+C

### Testing Sequence
1. ✅ **Mechanical Fixes** - Ensure free movement
2. ✅ **Calibration** - Find safe PWM limits
3. ✅ **Dry Run** - Verify CSV parsing without hardware
4. ✅ **Slow Test** - Low frame rate (10 Hz) with robot hanging
5. ✅ **Full Speed** - 50 Hz with monitoring
6. ✅ **Safety Test** - Disconnect PC, verify watchdog stops servos

---

## 📊 CSV Format

Expected format from `output_angles.csv`:

```csv
timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle
2025-10-07 17:59:35.950,1,173.74,166.63,170.66,170.33,168.96,170.67
2025-10-07 17:59:36.110,2,175.37,169.67,172.09,171.79,169.68,170.53
...
```

**Required Columns:**
- `right_hip_angle`, `right_knee_angle`, `right_ankle_angle`
- `left_hip_angle`, `left_knee_angle`, `left_ankle_angle`

**Optional Columns:**
- `timestamp` - For timing reference
- `frame` - Frame number

---

## 🐛 Troubleshooting

### Servos not moving
- [ ] Check external power supply (5-6V, sufficient current)
- [ ] Verify common ground between Arduino and servo power
- [ ] Confirm I2C connections (SDA/SCL)
- [ ] Check PCA9685 address (default 0x40)
- [ ] Verify calibration loaded: `PRINT_CAL`

### Servo jittering
- [ ] Increase `--filter-alpha` (more smoothing)
- [ ] Reduce frame rate: `--rate 30`
- [ ] Check mechanical friction
- [ ] Verify PWM limits not too narrow

### Reversed movement
- [ ] Set `invert: true` in `calib.json`
- [ ] Update via `CAL` command: `CAL,<joint_id>,<min>,<max>,1`

### Watchdog timeout
- [ ] Reduce frame rate if PC can't keep up
- [ ] Check USB cable quality
- [ ] Increase `WATCHDOG_TIMEOUT_MS` in Arduino code

### Python serial connection fails
- [ ] Install pyserial: `pip install pyserial`
- [ ] List ports: `python pose_to_arduino.py --list-ports`
- [ ] Close Serial Monitor before running Python
- [ ] Try different baud rate: `--baud 9600`

---

## 📁 Project Structure

```
bionic-leg-mediapose-pipeline/
├── ArduinoRelativeMovement/
│   └── ArduinoRelativeMovement.ino    # Arduino controller
├── pose_to_arduino.py                 # Python CSV streamer
├── calib.json                         # Calibration configuration
├── output_angles.csv                  # Captured gait data
├── MAIN.md                            # Full specification
└── README_RELATIVE_MOVEMENT.md        # This file
```

---

## 🔄 Workflow Summary

1. **Capture Gait Data** → `output_angles.csv` (using MediaPose)
2. **Mechanical Setup** → Fix friction, ensure free movement
3. **Calibrate Servos** → Find PWM limits, update `calib.json`
4. **Upload Arduino Code** → `ArduinoRelativeMovement.ino`
5. **Load Calibration** → Send `CAL` commands to Arduino
6. **Test Dry Run** → `python pose_to_arduino.py --dry-run`
7. **Test Slow** → `--rate 10` with robot hanging
8. **Full Speed** → `--rate 50 --relative --loop`

---

## 📚 Technical Details

### Filtering (Low-Pass)
Exponential Moving Average:
```
filtered[n] = α × raw[n] + (1-α) × filtered[n-1]
```
Default α = 0.2 (80% history, 20% new)

### PWM Mapping
```
fraction = (angle - angle_min) / (angle_max - angle_min)
if inverted: fraction = 1 - fraction
pwm = pwm_min + fraction × (pwm_max - pwm_min)
pwm = clamp(pwm_min, pwm_max)
```

### Ramping
```
max_delta = 2 PWM units per cycle
update_rate = 50 Hz (20ms)
max_speed ≈ 100 PWM units/second
```

---

## 🤝 Contributing

Found a bug or have improvements? Issues and PRs welcome!

### Known Limitations
- Hip joints (M5, M6) require mechanical fixes before calibration
- Right knee (M4) calibration assumed from left knee - needs verification
- Frame interpolation is linear (cubic spline could be smoother)

---

## 📝 License

This project is part of the bionic-leg-mediapose-pipeline repository.

---

## 🙏 Acknowledgments

- Adafruit for PCA9685 library
- OpenCV MediaPose for pose estimation
- Arduino community for servo control examples

---

**Questions?** Check MAIN.md for full system specification or open an issue.

**Safety First!** Always test with robot hanging/unloaded before ground operation.
