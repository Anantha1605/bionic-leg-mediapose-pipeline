// opencv-csv -> safe servo controller (4 motors shown)
// loads saved pwm limits from EEPROM (same layout as calibrator)
// accepts serial angle rows and maps them to safe pwm with ramp+smooth
// commands: SETANGLE idx min max | PRINT | STOP | CENTER

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

#define SERVO_MIN 102
#define SERVO_MAX 512

const uint8_t motor_chans[4] = {0,1,2,3}; // motor1 .. motor4 mapping
const int eeprom_base = 0; // must match calibrator (4 motors * 4 bytes = 16 bytes)

int saved_min_pwm[4];
int saved_max_pwm[4];

// angle bounds (degrees) you set via SETANGLE; default full 0..180
float saved_min_angle[4] = {0.0,0.0,0.0,0.0};
float saved_max_angle[4] = {180.0,180.0,180.0,180.0};

// runtime arrays
int current_pwm[4];
int target_pwm[4];

// smoothing + ramping params
const int max_delta_per_loop = 3;      // maximum pwm steps per update
const float angle_ema_alpha = 0.35;    // smoothing on incoming angles (0..1)
float ema_angle[4];                    // smoothed input angles

// serial parsing
String inBuf = "";

void selectChannel(uint8_t c){Wire.beginTransmission(TCAADDR);Wire.write(1<<c);Wire.endTransmission();delay(2);}
void safeSetPWM(uint8_t ch,uint16_t on,uint16_t off){selectChannel(0);pwm.setPWM(ch,on,off);}

// load saved pwm values from eeprom (same layout as calibrator)
void load_pwm_from_eeprom(){
  int addr = eeprom_base;
  for(int i=0;i<4;i++){
    int lo = EEPROM.read(addr); addr++;
    int hi = EEPROM.read(addr); addr++;
    saved_min_pwm[i] = (hi<<8) | lo;
    lo = EEPROM.read(addr); addr++;
    hi = EEPROM.read(addr); addr++;
    saved_max_pwm[i] = (hi<<8) | lo;
    // safety clamp
    if(saved_min_pwm[i] < SERVO_MIN) saved_min_pwm[i] = SERVO_MIN;
    if(saved_max_pwm[i] > SERVO_MAX) saved_max_pwm[i] = SERVO_MAX;
    if(saved_min_pwm[i] > saved_max_pwm[i]){ // fallback defaults
      saved_min_pwm[i] = SERVO_MIN + 20;
      saved_max_pwm[i] = SERVO_MAX - 20;
    }
  }
}

// map angle (deg) -> pwm using saved angle range and saved pwm bounds
int angle_to_pwm(int motorIndex, float angle){
  float amin = saved_min_angle[motorIndex];
  float amax = saved_max_angle[motorIndex];
  if(amax <= amin){ // invalid range -> treat as full
    amin = 0.0; amax = 180.0;
  }
  // clamp angle to angle bounds
  if(angle < amin) angle = amin;
  if(angle > amax) angle = amax;
  // normalized in [-1..1], where -1 -> amin, +1 -> amax
  float midAngle = (amin + amax) / 2.0;
  float halfAngle = (amax - amin) / 2.0;
  float u = 0.0;
  if(halfAngle > 0.0001) u = (angle - midAngle) / halfAngle;
  if(u>1.0) u=1.0; if(u<-1.0) u=-1.0;
  int minp = saved_min_pwm[motorIndex];
  int maxp = saved_max_pwm[motorIndex];
  int midp = (minp + maxp)/2;
  int halfp = (maxp - minp)/2;
  int pwmv = midp + (int)round(u * halfp);
  if(pwmv < minp) pwmv = minp;
  if(pwmv > maxp) pwmv = maxp;
  return pwmv;
}

// ramp toward target by at most maxDelta
int rampPwm(int current, int target, int maxDelta){
  int diff = target - current;
  if(abs(diff) <= maxDelta) return target;
  return current + (diff>0?maxDelta:-maxDelta);
}

void print_status(){
  Serial.println(F("=== status ==="));
  for(int i=0;i<4;i++){
    Serial.print("m");Serial.print(i+1);
    Serial.print(" pwm_min=");Serial.print(saved_min_pwm[i]);
    Serial.print(" pwm_max=");Serial.print(saved_max_pwm[i]);
    Serial.print(" ang_min=");Serial.print(saved_min_angle[i]);
    Serial.print(" ang_max=");Serial.println(saved_max_angle[i]);
  }
  Serial.println(F("commands: SETANGLE idx min max | PRINT | STOP | CENTER | STREAM rows of angles (6 values)"));
  Serial.println(F("expected angle order per row: right_hip right_knee right_ankle left_hip left_knee left_ankle"));
}

void center_all(){
  for(int i=0;i<4;i++){
    int mid = (saved_min_pwm[i]+saved_max_pwm[i])/2;
    target_pwm[i] = mid;
  }
  Serial.println("center -> moving to midpoints");
}

void emergency_stop(){
  center_all();
  Serial.println("EMERGENCY STOP: centering all");
}

void setup(){
  Serial.begin(115200);
  Wire.begin();
  Serial.println("\n=== opencv-csv -> safe-servo controller ===");
  selectChannel(0);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(50);

  load_pwm_from_eeprom();
  // init current pwm = midpoints
  for(int i=0;i<4;i++){
    current_pwm[i] = (saved_min_pwm[i] + saved_max_pwm[i]) / 2;
    target_pwm[i] = current_pwm[i];
    ema_angle[i] = (saved_min_angle[i] + saved_max_angle[i]) / 2.0;
    safeSetPWM(motor_chans[i],0,current_pwm[i]);
  }
  print_status();
  Serial.println("ready. stream CSV rows over serial or use commands.");
}

void apply_angles_to_targets(float angles[6]){
  // map the 6 input angles to our 4 motors in the naming convention:
  // mapping provided by user earlier: motor1 -> left ankle, motor2 -> right ankle, motor3 -> left knee, motor4 -> right knee
  // BUT your CSV order is: right_hip right_knee right_ankle left_hip left_knee left_ankle
  // so we must pick the corresponding indexes:
  // right_ankle = angles[2], left_ankle = angles[5], left_knee = angles[4], right_knee = angles[1]
  // motor1 = left_ankle -> angles[5]
  // motor2 = right_ankle -> angles[2]
  // motor3 = left_knee -> angles[4]
  // motor4 = right_knee -> angles[1]
  float inputForMotor[4];
  inputForMotor[0] = angles[5]; // motor1 left ankle
  inputForMotor[1] = angles[2]; // motor2 right ankle
  inputForMotor[2] = angles[4]; // motor3 left knee
  inputForMotor[3] = angles[1]; // motor4 right knee

  // apply EMA smoothing on angles (to reduce noise)
  for(int i=0;i<4;i++){
    ema_angle[i] = angle_ema_alpha * inputForMotor[i] + (1.0 - angle_ema_alpha) * ema_angle[i];
    int pwmv = angle_to_pwm(i, ema_angle[i]);
    target_pwm[i] = pwmv;
  }
}

void handle_line(String line){
  line.trim();
  if(line.length()==0) return;
  // check commands first
  if(line.startsWith("SETANGLE")){
    // format: SETANGLE idx min max
    int idx=0; float amin=0,amax=180;
    // parse tokens
    int p1 = line.indexOf(' ');
    if(p1>0){
      String rest = line.substring(p1+1);
      rest.trim();
      int p2 = rest.indexOf(' ');
      if(p2>0){
        String si = rest.substring(0,p2); si.trim();
        idx = si.toInt(); // 1..4 expected
        int p3 = rest.indexOf(' ', p2+1);
        String smin, smax;
        if(p3>0){
          smin = rest.substring(p2+1, p3);
          smax = rest.substring(p3+1);
        } else {
          smin = rest.substring(p2+1);
          smax = "";
        }
        smin.trim(); smax.trim();
        amin = smin.toFloat();
        amax = smax.toFloat();
        if(idx>=1 && idx<=4){
          saved_min_angle[idx-1] = amin;
          saved_max_angle[idx-1] = amax;
          Serial.print("SETANGLE -> m");Serial.print(idx);Serial.print(" ");Serial.print(amin);Serial.print(" ");Serial.println(amax);
        } else Serial.println("SETANGLE idx must be 1..4");
      }
    }
    return;
  } else if(line.equalsIgnoreCase("PRINT")){
    print_status(); return;
  } else if(line.equalsIgnoreCase("STOP") || line.equalsIgnoreCase("S")){
    emergency_stop(); return;
  } else if(line.equalsIgnoreCase("CENTER")){
    center_all(); return;
  }

  // else: parse as CSV row of angles; accept comma or tab separated values
  // we expect at least 6 numeric fields somewhere in the line
  float vals[6];
  for(int i=0;i<6;i++) vals[i]=0.0;

  // replace tabs with commas to unify
  for(unsigned int i=0;i<line.length();i++) if(line[i]=='\t') line[i]=',';
  // remove timestamp if present (first token is timestamp if contains ':')
  int firstComma = line.indexOf(',');
  if(firstComma>0){
    String firstTok = line.substring(0, firstComma);
    bool hasColon=false;
    for(unsigned int k=0;k<firstTok.length();k++) if(firstTok[k]==':') hasColon=true;
    if(hasColon){
      // drop first token
      line = line.substring(firstComma+1);
    }
  }
  // now split by commas
  int field=0;
  int start=0;
  while(field<6 && start < (int)line.length()){
    int cpos = line.indexOf(',', start);
    String tok;
    if(cpos == -1) { tok = line.substring(start); start = line.length(); }
    else { tok = line.substring(start, cpos); start = cpos + 1; }
    tok.trim();
    // sometimes frame column exists; skip non-numeric if needed
    if(tok.length()==0) { continue; }
    // try to convert to float if looks numeric
    bool looksNumeric=false;
    for(unsigned int k=0;k<tok.length();k++){
      char ch = tok[k];
      if((ch>='0' && ch<='9') || ch=='-' || ch=='+' || ch=='.' || ch=='e' || ch=='E') { looksNumeric=true; break; }
    }
    if(!looksNumeric) continue;
    // convert
    float v = tok.toFloat();
    vals[field]=v;
    field++;
  }
  if(field >= 6){
    // ok, we have 6 angles; process
    apply_angles_to_targets(vals);
  } else {
    // not enough numeric fields; ignore or print debug
    Serial.print("ignored line (need 6 numeric fields): ");
    Serial.println(line);
  }
}

unsigned long lastLoop = 0;
void loop(){
  // serial reading
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){
      if(inBuf.length()>0){
        handle_line(inBuf);
        inBuf="";
      }
    } else {
      inBuf += c;
      // guard long lines
      if(inBuf.length()>400) { inBuf=""; }
    }
  }

  // ramp current_pwm toward target_pwm with max_delta_per_loop
  for(int i=0;i<4;i++){
    int next = rampPwm(current_pwm[i], target_pwm[i], max_delta_per_loop);
    if(next != current_pwm[i]){
      current_pwm[i] = next;
      safeSetPWM(motor_chans[i], 0, current_pwm[i]);
    }
  }

  // small delay; you can tune loop frequency
  delay(15);
}
