# 🦿 Full Specification for Real-Time Robot Leg Control  
## Using OpenCV Angle Output → Python → UDP → Arduino → PCA9685  
### (Claude — Please read this entire markdown file and generate the complete system)

---

# 🎯 SYSTEM GOAL

Build a fully working system that moves a **6-DOF biped robot leg** (Left Hip/Knee/Ankle + Right Hip/Knee/Ankle) **based on angles coming from OpenCV**.

The CSV data looks like:

timestamp,frame,right_hip_angle,right_knee_angle,right_ankle_angle,left_hip_angle,left_knee_angle,left_ankle_angle
2025-10-07 17:59:35.950,1,173.7477,166.6367,170.6650,170.3326,168.9665,170.6764
2025-10-07 17:59:36.110,2,175.3778,169.6733,172.0928,171.7925,169.6825,170.5376

markdown
Copy code

These angles must drive **6 servo motors** safely through Arduino + PCA9685.

We must generate:

- **Python program** (reads CSV → Denoise → Map → UDP send)
- **Arduino program** (Receives → Map → Ramp-limit → PCA9685 outputs)
- **Calibration system**
- **Safety layer**
- **Testing workflow**
- **README + calib.json template**

---

# 🧱 SYSTEM ARCHITECTURE

## ✔ PC SIDE (Python → UDP)
Your Python script must:

1. Read CSV frames continuously or simulate real-time streaming.
2. Resample frames to 50Hz.
3. Interpolate between frames.
4. Low-pass filter angles.
5. Convert angles → PWM using calibration file.
6. Send UDP JSON packets to Arduino at 50Hz.
7. Provide calibration mode to generate angle↔pwm mapping.

---

## ✔ ARDUINO SIDE (UDP → PCA9685)
Arduino must:

1. Receive UDP packets (or Serial fallback).
2. Parse JSON containing 6 joint angles.
3. Convert angles → PWM using EEPROM calibration.
4. Ramp-limit PWM changes to avoid servo jerks.
5. Enforce hard PWM clamps.
6. Control PCA9685 via TCA9548A.
7. Implement watchdog (500ms timeout).
8. Accept Serial commands:
   - `STOP`
   - `CENTER`
   - `SAVE_CAL`
   - `LOAD_CAL`
   - `STATUS`

---

# 📐 CALIBRATION SYSTEM (CRITICAL)

Each joint needs a mapping:

angle_min
angle_max
pwm_min
pwm_max
invert (bool)

yaml
Copy code

Mapping formula:

frac = (angle - angle_min) / (angle_max - angle_min)
if invert == true: frac = 1 - frac
pwm = pwm_min + frac * (pwm_max - pwm_min)
pwm = clamp(pwm_min, pwm_max)

cpp
Copy code

Calibration file format (`calib.json`):

```json
{
  "r_hip":   {"angle_min": 0, "angle_max": 180, "pwm_min": 300, "pwm_max": 450, "invert": false},
  "r_knee":  {"angle_min": 0, "angle_max": 180, "pwm_min": 280, "pwm_max": 500, "invert": true},
  "r_ankle": {"angle_min": 0, "angle_max": 180, "pwm_min": 290, "pwm_max": 320, "invert": false},
  "l_hip":   {...},
  "l_knee":  {...},
  "l_ankle": {...}
}
Python Calibration Mode:
Slowly sweep PWM.

Read OpenCV angle at each PWM.

Build the linear relationship.

Save into calib.json.

Arduino Calibration Mode:
Load/save calibration from EEPROM.

Move joints to midpoints.

📡 UDP PACKET FORMAT
Python sends → Arduino:

json
Copy code
{
  "frame": 123,
  "angles": {
    "r_hip":173.74,
    "r_knee":166.63,
    "r_ankle":170.66,
    "l_hip":170.33,
    "l_knee":168.96,
    "l_ankle":170.67
  }
}
Update rate: 50 Hz (every 20 ms).

🔌 PCA9685 JOINT MAPPING
nginx
Copy code
PWM0 → r_hip
PWM1 → r_knee
PWM2 → r_ankle
PWM3 → l_hip
PWM4 → l_knee
PWM5 → l_ankle
Use:

TCA9548A channel = 0

PCA9685 frequency = 50Hz

Oscillator = 27MHz

🛡 SAFETY REQUIREMENTS
Arduino must enforce:
HARD PWM clamps from calibration.

Ramp limit changes (max Δ = 2 pwm per cycle).

Watchdog timeout = 500ms → STOP & CENTER.

Ignore any malformed packets.

Handle STOP and CENTER commands from Serial.

Python must:
Low-pass filter angles (α = 0.2).

Interpolate between frames.

Drop NaN or invalid angles.

Provide emergency STOP UDP packet.

🐍 PYTHON PROGRAM REQUIREMENTS
File: pose_to_udp.py
CLI:
css
Copy code
--csv <file>
--ip <arduino ip>
--port 5006
--rate <hz>
--calib <calib.json>
--invert-joint <joint>
--dry-run
--log debug
Must include:
CSV reader (simulated streaming).

Frame interpolator.

Exponential filter (alpha=0.2).

Angle→PWM conversion using calib.json.

UDP packet sender.

Calibration sweep routine.

Logging + debug printing.

Graceful stop on Ctrl+C.

🧩 ARDUINO PROGRAM REQUIREMENTS
File: udp_servo_receiver.ino
Must include:

Networking
UDP receive listener on port 5006 (or Serial fallback version included).

JSON Parsing
Extract only the 6 joint angles.

Servo Output
Convert angle → PWM via calibration.

Ramp limit each PWM value.

Output to PCA9685 channels correctly.

Commands over Serial:
STOP

CENTER

SAVE_CAL

LOAD_CAL

STATUS

Safety
Watchdog:

If no packet for 500ms → STOP.

Calibration:

Save/load from EEPROM.

Hard PWM clamps.

Loop Rate
Every 20 ms (50Hz).

🧪 TESTING SEQUENCE (Add to README)
Mechanical Fixes

Enlarge hip servo horn holes.

Reduce knee hinge friction.

Ensure servos freely move joints.

Calibration Sweeps

For each of 6 joints:

Sweep PWM slowly.

Record angle.

Generate calibration ranges.

Save to calib.json.

Dry Run

Run Python with --dry-run.

Integration

Run robot hanging freely.

Send CSV frames slowly.

Inspect joint mapping behavior.

Full Speed

Move to 50Hz.

Confirm no jitter.

Safety Tests

Disconnect PC → Arduino should STOP.

Send invalid angles → clamp.

Hit STOP → joints center.

📦 CLAUDE — WHAT YOU MUST GENERATE
Please output ALL of the following:

✅ 1. pose_to_udp.py
A complete Python script implementing:

CSV → processed angles → UDP JSON

Calibration routines

CLI

Logging

Filtering

Mapping

Interpolation

Safety handling

✅ 2. udp_servo_receiver.ino
A full Arduino program implementing:

UDP → JSON → angle mapping → PWM

EEPROM storage

PCA9685 control

TCA channel selection

Ramp limiting

Watchdog

Serial commands

Safety clamps

✅ 3. calib.json template
✅ 4. README.md documentation
Include:

Setup instructions

Wiring instructions

Testing flow

Calibration details

Troubleshooting

✅ 5. Calibration sweep helper code inside Python
✅ 6. All support code (helpers, utilities) required
All code must be:
Clean

Tested

Commented

Real-world usable

Not pseudo-code

ESSENTIAL DATA FOR CLAUDE (DO NOT IGNORE)
Robot hardware, joints, PWM limits, inversion, and channel mapping

These are the things ONLY we discovered together while testing — Claude will NOT know these unless you give it this list.

⚙️ PCA9685 Channel → Joint Mapping
M1 → Left Ankle  → PCA9685 Channel 0
M2 → Right Ankle → PCA9685 Channel 1
M3 → Left Knee   → PCA9685 Channel 2
M4 → Right Knee  → PCA9685 Channel 3
M5 → Left Hip    → PCA9685 Channel 4
M6 → Right Hip   → PCA9685 Channel 5

🧪 Servo MIN/MAX PWM Limits

These limits were found through careful slow sweeping + physical stop detection.

Use these for mapping angle → PWM (or direct PWM control).
No movement should exceed these safe values.

✅ Motor 1 — Left Ankle
MIN = 292
MAX = 324


Direction: Forward = increasing PWM
(i.e., PWM++ moves ankle upward)

✅ Motor 2 — Right Ankle
MIN = 291  (use this, not 292)
MAX = 325


Direction: Forward = decreasing PWM
(i.e., PWM-- moves it upward)

(Motor2 is inverted compared to motor1)

✅ Motor 3 — Left Knee

(Inversion like Motor2)

MIN = 309
MAX = 324


Direction: Forward = decreasing PWM

⚠️ Motor 4 — Right Knee

Same behavior as Motor3, but YOU STILL NEED TO CALIBRATE EXACT VALUES because mechanical friction prevents clean limits.

Tell Claude to assume:

Right Knee behaves like Left Knee (inversion same),
but exact MIN/MAX must be calibrated by the user.

⚠️ Motors 5 & 6 — Hips

Not calibrated yet because:

Hip servo horn hole is slightly too small

Leg does not fully attach

Cannot sweep safely

Calibration pending until mechanical fix

Tell Claude:

“Hip joints M5 and M6 have no safe MIN/MAX yet — code must allow later calibration.”

🔁 Servo Movement Rules Learned

These rules are crucial for Claude:

Rule 1 — PWM units must move slowly (ramping)
max change per loop: 1–2 PWM counts
loop rate: 20–50 ms

Rule 2 — Hard clamp

Code must clamp PWM to safe MIN/MAX values ALWAYS.

Rule 3 — Output must compensate for inversion

Examples:

M1 forward = PWM++

M2 forward = PWM--

M3 forward = PWM--

M4 likely same as M3 (not confirmed)

Rule 4 — PCA9685 settings
Frequency = 50 Hz
Oscillator = 27 MHz
Using TCA9548A channel 0

🧩 Calibration Status Summary

Give Claude this table:

Joint          | Channel | Min PWM | Max PWM | Inverted?
---------------------------------------------------------
Left Ankle     |   0     |   292   |   324   |   No
Right Ankle    |   1     |   291   |   325   |   Yes
Left Knee      |   2     |   309   |   324   |   Yes
Right Knee     |   3     |   ???   |   ???   |   Yes (assumed)
Left Hip       |   4     |  TBD    |  TBD    |  Unknown
Right Hip      |   5     |  TBD    |  TBD    |  Unknown