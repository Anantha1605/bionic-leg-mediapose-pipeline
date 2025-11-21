// =========================================
//  MOTOR-1 (Left Ankle) Auto Sweep Test
//  FINAL LIMITS FROM CALIBRATION:
//        MIN = 290
//        MAX = 332
//  Very slow and safe.
// =========================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm;

const uint8_t downstreamChannel = 0;
const uint8_t MOTOR1_CH = 0;

// YOUR NEW CALIBRATED LIMITS
const int M1_MIN = 292;
const int M1_MAX = 324;

// slow sweep settings
const int STEP = 1;                  // 1 PWM count per step
const unsigned long STEP_MS = 200;   // delay per step (very slow)

int current_pwm = (M1_MIN + M1_MAX) / 2;
bool goingUp = true;  // because you calibrated MIN → sweep+ → MAX

void selectChannel(uint8_t c){
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << c);
  Wire.endTransmission();
  delay(2);
}

void safeSetPWM(uint8_t ch, uint16_t on, uint16_t off){
  selectChannel(downstreamChannel);
  pwm.setPWM(ch, on, off);
}

void setup(){
  Serial.begin(115200);
  Wire.begin();

  pwm = Adafruit_PWMServoDriver();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(50);

  // safety: ensure midpoint inside range
  if(current_pwm < M1_MIN) current_pwm = M1_MIN;
  if(current_pwm > M1_MAX) current_pwm = M1_MAX;

  Serial.println("\n=== MOTOR-1 AUTO SWEEP START ===");
  Serial.print("Limits: "); Serial.print(M1_MIN); Serial.print(" .. "); Serial.println(M1_MAX);
  Serial.print("Start PWM = "); Serial.println(current_pwm);

  safeSetPWM(MOTOR1_CH, 0, current_pwm);
  delay(600);
}

unsigned long lastStep = 0;

void loop(){
  unsigned long now = millis();
  if(now - lastStep < STEP_MS) return;
  lastStep = now;

  if(goingUp){
    current_pwm += STEP;

    if(current_pwm >= M1_MAX){
      current_pwm = M1_MAX;
      Serial.println("Reached MAX → reversing");
      goingUp = false;
      safeSetPWM(MOTOR1_CH, 0, current_pwm);
      delay(200);
      return;
    }
  } else {
    current_pwm -= STEP;

    if(current_pwm <= M1_MIN){
      current_pwm = M1_MIN;
      Serial.println("Reached MIN → reversing");
      goingUp = true;
      safeSetPWM(MOTOR1_CH, 0, current_pwm);
      delay(200);
      return;
    }
  }

  // clamp again
  if(current_pwm < M1_MIN) current_pwm = M1_MIN;
  if(current_pwm > M1_MAX) current_pwm = M1_MAX;

  safeSetPWM(MOTOR1_CH, 0, current_pwm);

  Serial.print("M1 PWM = ");
  Serial.println(current_pwm);
}
