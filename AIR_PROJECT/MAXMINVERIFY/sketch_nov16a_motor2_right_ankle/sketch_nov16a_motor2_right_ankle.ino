// =========================================
//   MOTOR-2 SAFE AUTO-TEST (ANKLE RIGHT)
//   Uses your exact calibrated limits:
//      MIN = 285
//      MAX = 326
//   No Serial input needed.
//   Servo moves ultra-slow within limits.
// =========================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PCA9685 channel for Motor-2
const uint8_t downstreamChannel = 0;
const uint8_t MOTOR2_CH = 3;

// Your calibrated safe limits
const float motor2_min = 291;
const int motor2_max = 325;

// Ultra-slow movement tuning
const int step_pwm   = 1;      // move 1 PWM count per step
const int step_delay = 160;    // ms delay between steps (VERY slow)

int current_pwm = (motor2_min + motor2_max) / 2;  // start at midpoint
bool goingUp = true;  // sweep direction

void selectChannel(uint8_t c) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << c);
  Wire.endTransmission();
  delay(2);
}

void safeSetPWM(uint8_t ch, uint16_t on, uint16_t off) {
  selectChannel(downstreamChannel);
  pwm.setPWM(ch, on, off);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\n=== MOTOR-2 AUTO-TEST START ===");
  Serial.print("Safe range: ");
  Serial.print(motor2_min);
  Serial.print(" .. ");
  Serial.println(motor2_max);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(50);

  // Move to starting midpoint
  safeSetPWM(MOTOR2_CH, 0, current_pwm);
  Serial.print("Start at midpoint PWM=");
  Serial.println(current_pwm);
  delay(800);
}

unsigned long lastStep = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastStep >= step_delay) {
    lastStep = now;

    // Move within safe limits only
    if (goingUp) {
      if (current_pwm < motor2_max) {
        current_pwm += step_pwm;
      } else {
        goingUp = false;
        Serial.println("Reached MAX → reversing");
      }
    } else {
      if (current_pwm > motor2_min) {
        current_pwm -= step_pwm;
      } else {
        goingUp = true;
        Serial.println("Reached MIN → reversing");
      }
    }

    // Absolute safety clamp
    if (current_pwm < motor2_min) current_pwm = motor2_min;
    if (current_pwm > motor2_max) current_pwm = motor2_max;

    // Command servo
    safeSetPWM(MOTOR2_CH, 0, current_pwm);

    // Print current value
    Serial.print("M2 PWM=");
    Serial.println(current_pwm);
  }
}
