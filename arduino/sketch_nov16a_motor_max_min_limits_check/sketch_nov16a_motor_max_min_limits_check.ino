// ULTRA-SAFE servo-limit-calibrator - extremely slow sweeping
// Karthik Edition: tiny steps + huge delay so you can stop EXACTLY at limit
// motors: 1=LeftAnkle, 2=RightAnkle, 3=LeftKnee, 4=RightKnee

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 102
#define SERVO_CENTER 307
#define SERVO_MAX 512

// ultra slow sweep tuning
const int step_pwm   = 1;     // smallest step possible
const int step_delay = 200;   // <--- WAS 40. NOW 120ms (very slow)

// reduce serial spam
const bool SLOW_PRINT_MODE = true;

int neutralPulse1 = SERVO_CENTER;
int neutralPulse2 = SERVO_CENTER;
int neutralPulse3 = SERVO_CENTER;
int neutralPulse4 = SERVO_CENTER;

const uint8_t downstreamChannel = 0;
const uint8_t motor_chans[4] = {0,1,2,3};

// eeprom
const int eeprom_base = 0;
int saved_min_pwm[4];
int saved_max_pwm[4];

int selected = 0;
int live_pwm[4];

void selectChannel(uint8_t c){
  Wire.beginTransmission(TCAADDR);
  Wire.write(1<<c);
  Wire.endTransmission();
  delay(2);
}

void safeSetPWM(uint8_t ch, uint16_t on, uint16_t off){
  selectChannel(downstreamChannel);
  pwm.setPWM(ch, on, off);
}

void save_to_eeprom(){
  int addr = eeprom_base;
  for(int i=0;i<4;i++){
    EEPROM.update(addr, saved_min_pwm[i] & 0xFF); addr++;
    EEPROM.update(addr, (saved_min_pwm[i]>>8)&0xFF); addr++;
    EEPROM.update(addr, saved_max_pwm[i] & 0xFF); addr++;
    EEPROM.update(addr, (saved_max_pwm[i]>>8)&0xFF); addr++;
  }
  Serial.println("EEPROM saved!");
}

void load_from_eeprom(){
  int addr = eeprom_base;
  for(int i=0;i<4;i++){
    int lo = EEPROM.read(addr); addr++;
    int hi = EEPROM.read(addr); addr++;
    saved_min_pwm[i] = (hi<<8) | lo;

    lo = EEPROM.read(addr); addr++;
    hi = EEPROM.read(addr); addr++;
    saved_max_pwm[i] = (hi<<8) | lo;

    if(saved_min_pwm[i] < SERVO_MIN) saved_min_pwm[i] = SERVO_MIN;
    if(saved_max_pwm[i] > SERVO_MAX) saved_max_pwm[i] = SERVO_MAX;
    if(saved_min_pwm[i] > saved_max_pwm[i]){
      saved_min_pwm[i] = SERVO_MIN+20;
      saved_max_pwm[i] = SERVO_MAX-20;
    }
  }
  Serial.println("EEPROM loaded.");
}

void print_status(){
  Serial.println(F("===== ultra-slow calibrator ====="));
  Serial.print("Selected motor: "); Serial.println(selected+1);
  Serial.print("Live PWM: "); Serial.println(live_pwm[selected]);
  for(int i=0;i<4;i++){
    Serial.print("M"); Serial.print(i+1);
    Serial.print(" min="); Serial.print(saved_min_pwm[i]);
    Serial.print(" max="); Serial.println(saved_max_pwm[i]);
  }
  Serial.println(F("Commands:"));
  Serial.println(F("1-4 select motor"));
  Serial.println(F("r sweep+, l sweep- (VERY SLOW)"));
  Serial.println(F("m mark MIN, M mark MAX"));
  Serial.println(F("c center, a mid-all"));
  Serial.println(F("s stop sweep"));
  Serial.println(F("w write EEPROM, q load EEPROM"));
  Serial.println(F("p print status"));
}

void init_defaults(){
  for(int i=0;i<4;i++){
    saved_min_pwm[i] = SERVO_MIN + 20;
    saved_max_pwm[i] = SERVO_MAX - 20;
  }

  live_pwm[0]=neutralPulse1;
  live_pwm[1]=neutralPulse2;
  live_pwm[2]=neutralPulse3;
  live_pwm[3]=neutralPulse4;
}

void set_servo_pwm_idx(int idx, int pwmv){
  if(pwmv < SERVO_MIN) pwmv = SERVO_MIN;
  if(pwmv > SERVO_MAX) pwmv = SERVO_MAX;
  live_pwm[idx] = pwmv;
  safeSetPWM(motor_chans[idx], 0, pwmv);
}

void center_selected(){
  int mid = (saved_min_pwm[selected] + saved_max_pwm[selected]) / 2;
  set_servo_pwm_idx(selected, mid);
  Serial.print("Centered M"); Serial.println(selected+1);
}

void apply_all_mid(){
  for(int i=0;i<4;i++){
    int mid = (saved_min_pwm[i]+saved_max_pwm[i])/2;
    set_servo_pwm_idx(i, mid);
  }
  Serial.println("All to mid.");
}

bool sweeping = false;
int sweep_dir = 0;

void setup(){
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\n--- ULTRA SLOW CALIBRATOR START ---");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  init_defaults();
  set_servo_pwm_idx(0, neutralPulse1);
  set_servo_pwm_idx(1, neutralPulse2);
  set_servo_pwm_idx(2, neutralPulse3);
  set_servo_pwm_idx(3, neutralPulse4);

  load_from_eeprom();
  print_status();
}

void loop(){
  if(Serial.available()){
    char c = Serial.read();

    if(c>='1' && c<='4'){ selected=c-'1'; Serial.print("Selected M");Serial.println(selected+1); }

    else if(c=='r' || c=='R'){ sweeping=true; sweep_dir=+1; Serial.println("Sweep + (very slow)"); }
    else if(c=='l' || c=='L'){ sweeping=true; sweep_dir=-1; Serial.println("Sweep - (very slow)"); }

    else if(c=='s' || c=='S'){ sweeping=false; Serial.println("Sweep stopped."); }

    else if(c=='m'){ saved_min_pwm[selected]=live_pwm[selected]; Serial.println("Marked MIN."); }
    else if(c=='M'){ saved_max_pwm[selected]=live_pwm[selected]; Serial.println("Marked MAX."); }

    else if(c=='c'){ center_selected(); }
    else if(c=='a'){ apply_all_mid(); }

    else if(c=='w'){ save_to_eeprom(); }
    else if(c=='q'){ load_from_eeprom(); }
    else if(c=='p'){ print_status(); }
  }

  if(sweeping){
    int np = live_pwm[selected] + sweep_dir*step_pwm;
    if(np < SERVO_MIN) np = SERVO_MIN;
    if(np > SERVO_MAX) np = SERVO_MAX;

    set_servo_pwm_idx(selected,np);

    static unsigned long lastPrint=0;
    if(SLOW_PRINT_MODE){
      unsigned long t=millis();
      if(t-lastPrint>350){
        Serial.print("M");Serial.print(selected+1);Serial.print(" pwm=");Serial.println(live_pwm[selected]);
        lastPrint=t;
      }
    }

    delay(step_delay);
  } else {
    delay(15);
  }
}
