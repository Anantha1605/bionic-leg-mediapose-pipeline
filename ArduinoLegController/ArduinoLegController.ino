/*
  ArduinoLegController.ino
  Controls three MG996 servos (hip, knee, ankle) via Serial commands.

  Command format (one line, newline-terminated):
    <Side>,<hip_angle>,<knee_angle>,<ankle_angle>\n
  Side: R or L (kept for compatibility with CSV uploader)
  Angles: degrees as floats or integers. Values >180 mapped to 0-180 range by halving

  Example:
    R,90,45,120\n
  Safety notes:
  - Use an external 5V power supply capable of supplying servo stall current.
  - Connect servo ground to Arduino GND.
  - Keep per-servo angle limits within mechanical safe range.

  Simple smoothing is applied when moving between target angles.
  After reaching targets the board replies with "OK\n".
*/

#include <Servo.h>
#include <EEPROM.h>

// Pin assignments (change if needed)
// Right leg: servo IDs 0..2 -> R_hip, R_knee, R_ankle
// Left  leg: servo IDs 3..5 -> L_hip, L_knee, L_ankle
#define R_HIP_PIN   3
#define R_KNEE_PIN  5
#define R_ANKLE_PIN 6
#define L_HIP_PIN   9
#define L_KNEE_PIN 10
#define L_ANKLE_PIN 11

// Movement config
#define STEP_DELAY_MS 6     // delay per 1-degree step (smoothing)
#define SAFE_MIN_ANGLE 0
#define SAFE_MAX_ANGLE 180

Servo servos[6];

// Current positions per servo
float currentPos[6];

// Calibration stored in EEPROM: for each servo store float offset and a single byte for invert flag
// EEPROM layout: for i in 0..5: float offset (4 bytes) then 1 byte invert flag (0/1) => 5 * 6 = 30 bytes
#define EEPROM_BASE 0

unsigned long lastCommandMs = 0;
const unsigned long WATCHDOG_MS = 2000; // hold if no commands in this time

// Default calibration
float calOffset[6];
bool calInvert[6];

void loadCalibration() {
  int addr = EEPROM_BASE;
  for (int i = 0; i < 6; ++i) {
    float v = 0.0;
    EEPROM.get(addr, v);
    addr += sizeof(float);
    calOffset[i] = v;
    byte inv = 0;
    EEPROM.get(addr, inv);
    addr += 1;
    calInvert[i] = (inv != 0);
  }
}

void saveCalibration() {
  int addr = EEPROM_BASE;
  for (int i = 0; i < 6; ++i) {
    EEPROM.put(addr, calOffset[i]);
    addr += sizeof(float);
    byte inv = calInvert[i] ? 1 : 0;
    EEPROM.put(addr, inv);
    addr += 1;
  }
}

void setup() {
  // attach servos
  servos[0].attach(R_HIP_PIN);
  servos[1].attach(R_KNEE_PIN);
  servos[2].attach(R_ANKLE_PIN);
  servos[3].attach(L_HIP_PIN);
  servos[4].attach(L_KNEE_PIN);
  servos[5].attach(L_ANKLE_PIN);

  // initialize positions to neutral 90
  for (int i = 0; i < 6; ++i) {
    currentPos[i] = 90.0;
    servos[i].write((int)currentPos[i]);
  }

  // defaults
  for (int i = 0; i < 6; ++i) {
    calOffset[i] = 0.0;
    calInvert[i] = false;
  }

  loadCalibration();

  Serial.begin(115200);
}

float sanitizeAngle(float a) {
  if (isnan(a)) return 90.0;
  // If angle is in 0..360 and >180 assume it's a 0-360 reading; map to 0-180
  if (a > SAFE_MAX_ANGLE && a <= 360.0) {
    a = a * 0.5;
  }
  if (a < SAFE_MIN_ANGLE) a = SAFE_MIN_ANGLE;
  if (a > SAFE_MAX_ANGLE) a = SAFE_MAX_ANGLE;
  return a;
}

void moveServoSmoothIndex(int idx, float target) {
  target = sanitizeAngle(target);
  // apply calibration offset and invert
  float t = target + calOffset[idx];
  if (calInvert[idx]) t = 180.0 - t;
  if (t < SAFE_MIN_ANGLE) t = SAFE_MIN_ANGLE;
  if (t > SAFE_MAX_ANGLE) t = SAFE_MAX_ANGLE;

  int cur = (int)round(currentPos[idx]);
  int tgt = (int)round(t);
  if (cur == tgt) return;
  int step = (tgt > cur) ? 1 : -1;
  while (cur != tgt) {
    cur += step;
    servos[idx].write(cur);
    delay(STEP_DELAY_MS);
  }
  currentPos[idx] = tgt;
}

// Serial command protocol:
// - Stream command for one leg: L,<hip>,<knee>,<ankle>  (L = R or L)
// - Dual-leg stream: B,<rhip>,<rknee>,<rankle>,<lhip>,<lknee>,<lankle>
// - Calibration set: CAL,<servo_idx>,<offset_deg>,<invert_flag>  (invert_flag 0/1)  -> replies OK
// - Save calibration to EEPROM: SAVE_CAL -> replies OK
// - Dump calibration: DUMP_CAL -> replies calibration lines

void processLine(char *line) {
  // tokenize
  char *token = strtok(line, ",\r\n");
  if (!token) return;
  if (strcmp(token, "B") == 0) {
    // Both legs: expect 6 angles
    float vals[6];
    for (int i = 0; i < 6; ++i) {
      token = strtok(NULL, ",\r\n");
      if (!token) return;
      vals[i] = atof(token);
    }
    // Map: vals[0..2] -> right hip,knee,ankle (servos 0..2)
    for (int i = 0; i < 3; ++i) moveServoSmoothIndex(i, vals[i]);
    // Map: vals[3..5] -> left hip,knee,ankle (servos 3..5)
    for (int i = 0; i < 3; ++i) moveServoSmoothIndex(3 + i, vals[3 + i]);
    Serial.println("OK");
    lastCommandMs = millis();
    return;
  }

  if (strcmp(token, "R") == 0 || strcmp(token, "L") == 0) {
    bool isRight = (token[0] == 'R');
    float hip = atof(strtok(NULL, ",\r\n"));
    float knee = atof(strtok(NULL, ",\r\n"));
    float ankle = atof(strtok(NULL, ",\r\n"));
    int base = isRight ? 0 : 3;
    moveServoSmoothIndex(base + 0, hip);
    moveServoSmoothIndex(base + 1, knee);
    moveServoSmoothIndex(base + 2, ankle);
    Serial.println("OK");
    lastCommandMs = millis();
    return;
  }

  if (strcmp(token, "CAL") == 0) {
    // CAL,<servo_idx 0-5>,<offset float>,<invert 0/1>
    token = strtok(NULL, ",\r\n"); if (!token) return; int idx = atoi(token);
    token = strtok(NULL, ",\r\n"); if (!token) return; float off = atof(token);
    token = strtok(NULL, ",\r\n"); if (!token) return; int inv = atoi(token);
    if (idx >= 0 && idx < 6) {
      calOffset[idx] = off;
      calInvert[idx] = (inv != 0);
      Serial.println("OK");
    } else {
      Serial.println("ERR");
    }
    return;
  }

  if (strcmp(token, "SAVE_CAL") == 0) {
    saveCalibration();
    Serial.println("OK");
    return;
  }

  if (strcmp(token, "DUMP_CAL") == 0) {
    for (int i = 0; i < 6; ++i) {
      Serial.print("CAL,"); Serial.print(i); Serial.print(",");
      Serial.print(calOffset[i]); Serial.print(","); Serial.println(calInvert[i] ? 1 : 0);
    }
    Serial.println("OK");
    return;
  }
}

void loop() {
  static const int BUF_SZ = 192;
  static char buf[BUF_SZ];
  static int idx = 0;

  // Read Serial buffer and collect until newline
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buf[idx] = '\0';
        processLine(buf);
        idx = 0;
      }
    } else {
      if (idx < BUF_SZ - 1) buf[idx++] = c;
    }
  }

  // If no recent commands, keep servos at current positions (watchdog can be extended here)
  if (millis() - lastCommandMs > WATCHDOG_MS) {
    for (int i = 0; i < 6; ++i) servos[i].write((int)currentPos[i]);
    lastCommandMs = millis();
  }
}
