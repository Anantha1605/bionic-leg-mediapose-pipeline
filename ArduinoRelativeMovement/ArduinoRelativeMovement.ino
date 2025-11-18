/*
  ArduinoRelativeMovement.ino
  
  Real-time biped robot leg controller with RELATIVE MOVEMENT for smooth gait replication
  Based on angle data from OpenCV MediaPose → CSV → Python → Arduino
  
  Features:
  - PCA9685 servo driver control via I2C (TCA9548A multiplexer channel 0)
  - 6-DOF control (L/R Hip, Knee, Ankle)
  - RELATIVE movement mode: calculates delta from baseline position
  - Smooth ramping with adjustable speed limits
  - Safety: Hard PWM clamps, watchdog timeout, emergency stop
  - Serial command protocol for calibration and control
  - EEPROM calibration storage
  
  Hardware:
  - Arduino compatible board (Uno, Mega, ESP32, etc.)
  - PCA9685 16-channel servo driver (I2C address 0x40)
  - TCA9548A I2C multiplexer (optional, channel 0)
  - 6x servo motors
  
  Wiring:
  - SDA/SCL to TCA9548A
  - TCA9548A SD0/SC0 to PCA9685
  - PCA9685 PWM0-5 to servos M1-M6
  - Servos powered externally (5-6V, 5-10A capable)
  
  Serial Commands:
  - ANGLES,<r_hip>,<r_knee>,<r_ankle>,<l_hip>,<l_knee>,<l_ankle>  // Set target angles
  - BASELINE,<r_hip>,<r_knee>,<r_ankle>,<l_hip>,<l_knee>,<l_ankle>  // Set baseline reference
  - RELATIVE,<enabled>  // Enable/disable relative mode (1/0)
  - STOP  // Emergency stop - center all servos
  - CENTER  // Move all to neutral position
  - STATUS  // Print current state
  - CAL,<joint_id>,<pwm_min>,<pwm_max>,<invert>  // Set calibration
  - SAVE_CAL  // Save calibration to EEPROM
  - LOAD_CAL  // Load calibration from EEPROM
  
  Author: Generated for bionic-leg-mediapose-pipeline
  Date: 2025-11-18
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// ============= CONFIGURATION =============

// Debug options
// #define DEBUG_ANGLES      // Print received angles
// #define DEBUG_PWM         // Print PWM values every 500ms
#define DEBUG_STARTUP     // Print detailed startup info

// PCA9685 settings
#define PCA9685_ADDR 0x40
#define PCA9685_FREQ 50           // 50Hz for servos
#define OSCILLATOR_FREQ 27000000  // 27MHz oscillator

// TCA9548A multiplexer
#define TCA9548A_ADDR 0x70
#define TCA9548A_CHANNEL 0

// Servo channel mapping
#define LEFT_ANKLE_CH   0  // M1
#define RIGHT_ANKLE_CH  1  // M2
#define LEFT_KNEE_CH    2  // M3
#define RIGHT_KNEE_CH   3  // M4
#define LEFT_HIP_CH     4  // M5
#define RIGHT_HIP_CH    5  // M6

#define NUM_JOINTS 6

// Joint index mapping
#define R_HIP   0
#define R_KNEE  1
#define R_ANKLE 2
#define L_HIP   3
#define L_KNEE  4
#define L_ANKLE 5

// PWM ramp limiting - INCREASED FOR 30 FPS INPUT
#define MAX_PWM_DELTA_PER_CYCLE 20  // Was 2, now 20 for faster response
#define LOOP_DELAY_MS 20            // 50Hz update rate

// Watchdog timeout
#define WATCHDOG_TIMEOUT_MS 500

// Safety angle limits
#define SAFE_ANGLE_MIN 0.0
#define SAFE_ANGLE_MAX 180.0

// EEPROM addresses
#define EEPROM_MAGIC 0x42
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_CAL_START 1

// ============= GLOBAL VARIABLES =============

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Joint calibration data
struct JointCalibration {
  uint16_t pwmMin;
  uint16_t pwmMax;
  bool inverted;
  float angleMin;
  float angleMax;
};

// Channel mapping to joint index
const uint8_t channelMap[NUM_JOINTS] = {
  RIGHT_HIP_CH,    // 0: R_HIP
  RIGHT_KNEE_CH,   // 1: R_KNEE
  RIGHT_ANKLE_CH,  // 2: R_ANKLE
  LEFT_HIP_CH,     // 3: L_HIP
  LEFT_KNEE_CH,    // 4: L_KNEE
  LEFT_ANKLE_CH    // 5: L_ANKLE
};

// CORRECTED calibration values for MG996R servos
// PWM calculation: microseconds / 4.88 = PWM value
// 500μs = 102, 1500μs = 307, 2500μs = 512
JointCalibration jointCal[NUM_JOINTS] = {
  {150, 450, false, 0, 180},  // R_HIP (M6) - TBD, use safe range
  {150, 450, true,  0, 180},  // R_KNEE (M4) - TBD, assumed inverted
  {102, 512, true,  0, 180},  // R_ANKLE (M2) - Full range, inverted
  {150, 450, false, 0, 180},  // L_HIP (M5) - TBD, use safe range
  {102, 512, true,  0, 180},  // L_KNEE (M3) - Full range, inverted
  {102, 512, false, 0, 180}   // L_ANKLE (M1) - Full range
};

// Current state
float currentAngles[NUM_JOINTS];
uint16_t currentPWM[NUM_JOINTS];
uint16_t targetPWM[NUM_JOINTS];

bool systemActive = false;
unsigned long lastCommandTime = 0;
unsigned long frameCount = 0;

// ============= HELPER FUNCTIONS =============

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

float constrainFloat(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}

// Convert angle to PWM using calibration
uint16_t angleToPWM(uint8_t jointIdx, float angle) {
  JointCalibration &cal = jointCal[jointIdx];
  
  angle = constrainFloat(angle, cal.angleMin, cal.angleMax);
  float fraction = (angle - cal.angleMin) / (cal.angleMax - cal.angleMin);
  
  if (cal.inverted) {
    fraction = 1.0 - fraction;
  }
  
  uint16_t pwm = cal.pwmMin + (uint16_t)(fraction * (cal.pwmMax - cal.pwmMin));
  
  if (pwm < cal.pwmMin) pwm = cal.pwmMin;
  if (pwm > cal.pwmMax) pwm = cal.pwmMax;
  
  return pwm;
}

void setServoPWM(uint8_t jointIdx, uint16_t targetValue) {
  targetPWM[jointIdx] = targetValue;
}

// Update all servos with ramping
void updateServos() {
  tcaSelect(TCA9548A_CHANNEL);
  
  #ifdef DEBUG_PWM
  static unsigned long lastPrint = 0;
  bool shouldPrint = (millis() - lastPrint > 500);
  if (shouldPrint) {
    Serial.print(F("PWM: "));
  }
  #endif
  
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    uint16_t current = currentPWM[i];
    uint16_t target = targetPWM[i];
    
    // Ramp limiting
    if (current < target) {
      current += min((uint16_t)MAX_PWM_DELTA_PER_CYCLE, target - current);
    } else if (current > target) {
      current -= min((uint16_t)MAX_PWM_DELTA_PER_CYCLE, current - target);
    }
    
    currentPWM[i] = current;
    pwm.setPWM(channelMap[i], 0, current);
    
    #ifdef DEBUG_PWM
    if (shouldPrint) {
      Serial.print(current);
      Serial.print(F(" "));
    }
    #endif
  }
  
  #ifdef DEBUG_PWM
  if (shouldPrint) {
    Serial.println();
    lastPrint = millis();
  }
  #endif
}

void emergencyStop() {
  Serial.println(F("EMERGENCY STOP"));
  systemActive = false;
  
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    currentAngles[i] = 90.0;
    targetPWM[i] = angleToPWM(i, 90.0);
  }
  
  updateServos();
}

// Set all joints to specific angles - ABSOLUTE MODE ONLY
void setJointAngles(float angles[NUM_JOINTS]) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    float angle = constrainFloat(angles[i], SAFE_ANGLE_MIN, SAFE_ANGLE_MAX);
    
    // REMOVED: Relative mode logic (CSV contains absolute angles)
    
    currentAngles[i] = angle;
    uint16_t pwmValue = angleToPWM(i, angle);
    setServoPWM(i, pwmValue);
  }
  
  lastCommandTime = millis();
  systemActive = true;
  frameCount++;
}

// ============= CALIBRATION FUNCTIONS =============

void saveCalibration() {
  int addr = EEPROM_CAL_START;
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    EEPROM.put(addr, jointCal[i].pwmMin);
    addr += sizeof(uint16_t);
    EEPROM.put(addr, jointCal[i].pwmMax);
    addr += sizeof(uint16_t);
    EEPROM.put(addr, jointCal[i].inverted ? (uint8_t)1 : (uint8_t)0);
    addr += sizeof(uint8_t);
  }
  
  Serial.println(F("CAL_SAVED"));
}

void loadCalibration() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    Serial.println(F("CAL_DEFAULT"));
    return;
  }
  
  int addr = EEPROM_CAL_START;
  
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    EEPROM.get(addr, jointCal[i].pwmMin);
    addr += sizeof(uint16_t);
    EEPROM.get(addr, jointCal[i].pwmMax);
    addr += sizeof(uint16_t);
    uint8_t inv;
    EEPROM.get(addr, inv);
    jointCal[i].inverted = (inv != 0);
    addr += sizeof(uint8_t);
  }
  
  Serial.println(F("CAL_LOADED"));
}

void printCalibration() {
  const char* names[] = {"R_HIP", "R_KNEE", "R_ANKLE", "L_HIP", "L_KNEE", "L_ANKLE"};
  
  Serial.println(F("=== Current Calibration ==="));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Serial.print(names[i]);
    Serial.print(F(": PWM "));
    Serial.print(jointCal[i].pwmMin);
    Serial.print(F("-"));
    Serial.print(jointCal[i].pwmMax);
    Serial.print(F(" ("));
    Serial.print(jointCal[i].pwmMax - jointCal[i].pwmMin);
    Serial.print(F(" range) INV="));
    Serial.println(jointCal[i].inverted ? "YES" : "NO");
  }
}

// ============= COMMAND PARSING =============

void processCommand(char* cmd) {
  while (*cmd == ' ' || *cmd == '\t') cmd++;
  
  if (strncmp(cmd, "ANGLES,", 7) == 0) {
    float angles[NUM_JOINTS];
    char* ptr = cmd + 7;
    
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      angles[i] = atof(ptr);
      ptr = strchr(ptr, ',');
      if (ptr == NULL && i < NUM_JOINTS - 1) {
        Serial.println(F("ERR_PARSE"));
        return;
      }
      if (ptr) ptr++;
    }
    
    #ifdef DEBUG_ANGLES
    Serial.print(F("RX: "));
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      Serial.print(angles[i], 1);
      Serial.print(F(" "));
    }
    Serial.println();
    #endif
    
    setJointAngles(angles);
    Serial.println(F("OK"));  // ACK for Python synchronization
  }
  else if (strcmp(cmd, "STOP") == 0) {
    emergencyStop();
    Serial.println(F("OK"));
  }
  else if (strcmp(cmd, "CENTER") == 0) {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      currentAngles[i] = 90.0;
      targetPWM[i] = angleToPWM(i, 90.0);
    }
    Serial.println(F("CENTERED"));
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    Serial.print(F("ACTIVE="));
    Serial.print(systemActive ? "YES" : "NO");
    Serial.print(F(" FRAMES="));
    Serial.print(frameCount);
    Serial.print(F(" TIME="));
    Serial.println(millis() - lastCommandTime);
    
    const char* names[] = {"R_HIP", "R_KNEE", "R_ANKLE", "L_HIP", "L_KNEE", "L_ANKLE"};
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      Serial.print(names[i]);
      Serial.print(F("="));
      Serial.print(currentAngles[i], 1);
      Serial.print(F("° PWM="));
      Serial.print(currentPWM[i]);
      Serial.print(F(" "));
    }
    Serial.println();
  }
  else if (strncmp(cmd, "CAL,", 4) == 0) {
    char* ptr = cmd + 4;
    uint8_t jointId = atoi(ptr);
    
    if (jointId >= NUM_JOINTS) {
      Serial.println(F("ERR_JOINT_ID"));
      return;
    }
    
    ptr = strchr(ptr, ','); if (!ptr) { Serial.println(F("ERR_PARSE")); return; } ptr++;
    jointCal[jointId].pwmMin = atoi(ptr);
    
    ptr = strchr(ptr, ','); if (!ptr) { Serial.println(F("ERR_PARSE")); return; } ptr++;
    jointCal[jointId].pwmMax = atoi(ptr);
    
    ptr = strchr(ptr, ','); if (!ptr) { Serial.println(F("ERR_PARSE")); return; } ptr++;
    jointCal[jointId].inverted = (atoi(ptr) != 0);
    
    Serial.println(F("CAL_SET"));
  }
  else if (strcmp(cmd, "SAVE_CAL") == 0) {
    saveCalibration();
  }
  else if (strcmp(cmd, "LOAD_CAL") == 0) {
    loadCalibration();
  }
  else if (strcmp(cmd, "PRINT_CAL") == 0) {
    printCalibration();
  }
  else {
    Serial.println(F("ERR_UNKNOWN_CMD"));
  }
}

// ============= SETUP & LOOP =============

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  #ifdef DEBUG_STARTUP
  Serial.println(F("\n=== Arduino MG996R Controller FIXED ==="));
  Serial.println(F("Version 2.0 - Absolute Angle Mode"));
  Serial.println(F("Features:"));
  Serial.println(F("  - CSV absolute angles → direct servo control"));
  Serial.println(F("  - Corrected PWM calibration for MG996R"));
  Serial.println(F("  - Faster ramping (20 PWM/cycle @ 50Hz)"));
  Serial.println(F("  - 30 FPS CSV input compatible"));
  #endif
  
  Wire.begin();
  Wire.setClock(400000);
  
  tcaSelect(TCA9548A_CHANNEL);
  
  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_FREQ);
  pwm.setPWMFreq(PCA9685_FREQ);
  delay(10);
  
  loadCalibration();
  
  #ifdef DEBUG_STARTUP
  Serial.println(F("\nInitializing to center (90°)..."));
  printCalibration();
  #endif
  
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    currentAngles[i] = 90.0;
    currentPWM[i] = angleToPWM(i, 90.0);
    targetPWM[i] = currentPWM[i];
  }
  
  updateServos();
  
  Serial.println(F("\nREADY"));
  Serial.println(F("Commands: ANGLES, STOP, CENTER, STATUS, CAL, SAVE_CAL, LOAD_CAL, PRINT_CAL"));
}

void loop() {
  static char cmdBuffer[128];
  static uint8_t cmdIdx = 0;
  
  // Read serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuffer[cmdIdx] = '\0';
        processCommand(cmdBuffer);
        cmdIdx = 0;
      }
    } else if (cmdIdx < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIdx++] = c;
    }
  }
  
  updateServos();
  
  // Watchdog check
  if (systemActive && (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    emergencyStop();
  }
  
  delay(LOOP_DELAY_MS);
}