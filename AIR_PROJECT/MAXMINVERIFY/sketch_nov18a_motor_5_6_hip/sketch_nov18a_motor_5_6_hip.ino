// =========================================
//  MOTOR-5 & MOTOR-6 SAFE AUTO-SWEEP
//  Non-blocking, independent sweeps for two motors
//  Style matches Motor-1 / Motor-2 safe tests
// =========================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channels (change if your wiring differs)
const uint8_t MOTOR5_CH = 4;  // hip left? adjust if needed
const uint8_t MOTOR6_CH = 5;  // hip right? adjust if needed

// ====== REPLACE THESE WITH YOUR CALIBRATED LIMITS ======
// Use integer PWM counts (no floats). Round up for safety if unsure.
const int M5_MIN = 290;  // example placeholder - set to your measured min
const int M5_MAX = 330;  // example placeholder - set to your measured max

const int M6_MIN = 295;  // example placeholder - set to your measured min
const int M6_MAX = 328;  // example placeholder - set to your measured max
// =======================================================

// step and speed tuning (very slow by default)
const int STEP = 1;
const unsigned long STEP_MS_5 = 200; // ms per step motor5 (tune if you want faster)
const unsigned long STEP_MS_6 = 200; // ms per step motor6

// per-motor state
int m5_pwm; // current pwm
int m6_pwm;
enum Phase { TO_MAX, TO_MIN };
Phase m5_phase = TO_MAX;
Phase m6_phase = TO_MAX;

// timestamps for independent timing
unsigned long lastStep5 = 0;
unsigned long lastStep6 = 0;

void selectChannel(uint8_t c){
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << c);
  Wire.endTransmission();
  delay(2);
}

void safeSetPWM(uint8_t ch, uint16_t on, uint16_t off){
  // we always select TCA channel 0 (downstreamChannel) per your setup
  selectChannel(0);
  pwm.setPWM(ch, on, off);
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(50);

  // initialize current PWMs to midpoint inside each range
  m5_pwm = (M5_MIN + M5_MAX) / 2;
  if(m5_pwm < M5_MIN) m5_pwm = M5_MIN;
  if(m5_pwm > M5_MAX) m5_pwm = M5_MAX;

  m6_pwm = (M6_MIN + M6_MAX) / 2;
  if(m6_pwm < M6_MIN) m6_pwm = M6_MIN;
  if(m6_pwm > M6_MAX) m6_pwm = M6_MAX;

  // print header
  Serial.println("\n=== MOTOR-5 & MOTOR-6 AUTO-SWEEP ===");
  Serial.print("M5 limits: "); Serial.print(M5_MIN); Serial.print(" .. "); Serial.println(M5_MAX);
  Serial.print("M6 limits: "); Serial.print(M6_MIN); Serial.print(" .. "); Serial.println(M6_MAX);
  Serial.print("Starting M5 PWM = "); Serial.println(m5_pwm);
  Serial.print("Starting M6 PWM = "); Serial.println(m6_pwm);

  // command initial positions
  safeSetPWM(MOTOR5_CH, 0, m5_pwm);
  safeSetPWM(MOTOR6_CH, 0, m6_pwm);

  delay(600);
}

void loop(){
  unsigned long now = millis();

  // ----- MOTOR 5 handling -----
  if(now - lastStep5 >= STEP_MS_5){
    lastStep5 = now;

    if(m5_phase == TO_MAX){
      m5_pwm += STEP;
      if(m5_pwm >= M5_MAX){
        m5_pwm = M5_MAX;
        m5_phase = TO_MIN;
        Serial.println("M5 reached MAX -> reversing to MIN");
      }
    } else { // TO_MIN
      m5_pwm -= STEP;
      if(m5_pwm <= M5_MIN){
        m5_pwm = M5_MIN;
        m5_phase = TO_MAX;
        Serial.println("M5 reached MIN -> reversing to MAX");
      }
    }

    // enforce bounds (safety)
    if(m5_pwm < M5_MIN) m5_pwm = M5_MIN;
    if(m5_pwm > M5_MAX) m5_pwm = M5_MAX;

    safeSetPWM(MOTOR5_CH, 0, m5_pwm);
    Serial.print("M5 PWM="); Serial.println(m5_pwm);
  }

  // ----- MOTOR 6 handling -----
  if(now - lastStep6 >= STEP_MS_6){
    lastStep6 = now;

    if(m6_phase == TO_MAX){
      m6_pwm += STEP;
      if(m6_pwm >= M6_MAX){
        m6_pwm = M6_MAX;
        m6_phase = TO_MIN;
        Serial.println("M6 reached MAX -> reversing to MIN");
      }
    } else { // TO_MIN
      m6_pwm -= STEP;
      if(m6_pwm <= M6_MIN){
        m6_pwm = M6_MIN;
        m6_phase = TO_MAX;
        Serial.println("M6 reached MIN -> reversing to MAX");
      }
    }

    // enforce bounds (safety)
    if(m6_pwm < M6_MIN) m6_pwm = M6_MIN;
    if(m6_pwm > M6_MAX) m6_pwm = M6_MAX;

    safeSetPWM(MOTOR6_CH, 0, m6_pwm);
    Serial.print("M6 PWM="); Serial.println(m6_pwm);
  }

  // small yield to keep loop responsive
  // (no blocking calls here)
}
