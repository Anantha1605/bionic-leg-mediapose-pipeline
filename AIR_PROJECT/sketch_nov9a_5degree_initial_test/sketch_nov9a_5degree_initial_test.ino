// simultaneous 4-motor simple r/l control (safe)
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define TCAADDR 0x70
Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();

#define SERVO_MIN 102
#define SERVO_CENTER 307
#define SERVO_MAX 512

int neutralPulse1=SERVO_CENTER;
int neutralPulse2=SERVO_CENTER;
int neutralPulse3=SERVO_CENTER;
int neutralPulse4=SERVO_CENTER;

const int degrees_per_step=7;
int ms_per_degree=25;
const int motion_offset_counts=20;
const int ramp_steps=16;
const int ramp_delay=10;

const int max_duration_ms=700;
const int max_degrees_per_cmd=15;
const int min_neutral_limit=SERVO_MIN+15;
const int max_neutral_limit=SERVO_MAX-15;

const uint8_t downstreamChannel=0;
const uint8_t motor1_chan=0;
const uint8_t motor2_chan=1;
const uint8_t motor3_chan=2;
const uint8_t motor4_chan=3;

enum mstate_e{MS_IDLE,MS_RAMP_UP,MS_HOLD,MS_RAMP_DOWN};
struct Motor{
  uint8_t chan;int neutral;int current;int target;int dir;int rampIndex;unsigned long lastStep;unsigned long holdUntil;mstate_e state;
} motors[4];

void clamp_neutral(int &n){if(n<min_neutral_limit){n=min_neutral_limit;Serial.println("neutral hit min limit");}if(n>max_neutral_limit){n=max_neutral_limit;Serial.println("neutral hit max limit");}}

void selectChannel(uint8_t c){Wire.beginTransmission(TCAADDR);Wire.write(1<<c);Wire.endTransmission();delay(2);}
void safeSetPWM(uint8_t ch,uint16_t on,uint16_t off){selectChannel(downstreamChannel);pwm.setPWM(ch,on,off);}

void init_motors(){
  motors[0].chan=motor1_chan;motors[0].neutral=neutralPulse1;
  motors[1].chan=motor2_chan;motors[1].neutral=neutralPulse2;
  motors[2].chan=motor3_chan;motors[2].neutral=neutralPulse3;
  motors[3].chan=motor4_chan;motors[3].neutral=neutralPulse4;
  for(int i=0;i<4;i++){motors[i].current=motors[i].neutral;motors[i].target=motors[i].neutral;motors[i].dir=0;motors[i].rampIndex=0;motors[i].lastStep=0;motors[i].holdUntil=0;motors[i].state=MS_IDLE;safeSetPWM(motors[i].chan,0,motors[i].neutral);}
}

void start_move_idx(int idx,int dir,int degrees){
  if(idx<0||idx>3) return;
  if(dir==0){
    motors[idx].dir=0;motors[idx].target=motors[idx].neutral;motors[idx].rampIndex=ramp_steps;motors[idx].state=MS_RAMP_DOWN;motors[idx].lastStep=millis();return;
  }
  if(degrees<=0) return;
  if(degrees>max_degrees_per_cmd){degrees=max_degrees_per_cmd;Serial.print("deg clamped ");Serial.println(max_degrees_per_cmd);}
  long dur=(long)degrees*(long)ms_per_degree;
  if(dur>max_duration_ms){dur=max_duration_ms;Serial.print("dur clamped ");Serial.println(max_duration_ms);}
  int t=motors[idx].neutral+dir*motion_offset_counts;
  motors[idx].dir=dir;motors[idx].target=t;motors[idx].rampIndex=0;motors[idx].lastStep=millis();motors[idx].holdUntil=millis()+dur;motors[idx].state=MS_RAMP_UP;
}

void start_move_all(int dir,int degrees){
  for(int i=0;i<4;i++) start_move_idx(i,dir,degrees);
}

void cancel_all_and_center(){
  for(int i=0;i<4;i++){motors[i].dir=0;motors[i].target=motors[i].neutral;motors[i].rampIndex=ramp_steps;motors[i].state=MS_RAMP_DOWN;motors[i].lastStep=millis();}
  Serial.println("allstop");
}

void process_motor(Motor &m){
  unsigned long now=millis();
  if(m.state==MS_IDLE) return;
  int diff=m.target-m.neutral;
  if(m.state==MS_RAMP_UP){
    if(now-m.lastStep>=ramp_delay){
      m.rampIndex++; if(m.rampIndex>ramp_steps) m.rampIndex=ramp_steps;
      m.current=m.neutral+(diff*m.rampIndex)/ramp_steps;
      safeSetPWM(m.chan,0,m.current);
      m.lastStep=now;
      if(m.rampIndex>=ramp_steps){ m.state=MS_HOLD; }
    }
  }else if(m.state==MS_HOLD){
    safeSetPWM(m.chan,0,m.target);
    if(now>=m.holdUntil){ m.state=MS_RAMP_DOWN; m.rampIndex=ramp_steps; m.lastStep=now; }
  }else if(m.state==MS_RAMP_DOWN){
    if(now-m.lastStep>=ramp_delay){
      m.rampIndex--; if(m.rampIndex<0) m.rampIndex=0;
      m.current=m.neutral+(diff*m.rampIndex)/ramp_steps;
      safeSetPWM(m.chan,0,m.current);
      m.lastStep=now;
      if(m.rampIndex<=0){ m.state=MS_IDLE; safeSetPWM(m.chan,0,m.neutral); Serial.print("ch");Serial.print(m.chan);Serial.println(": idle->neutral"); }
    }
  }
}

String inBuf="";
void setup(){
  Serial.begin(9600);Wire.begin();
  Serial.println("4-motor r/l simple (r=forward l=back s/0=center)");
  selectChannel(downstreamChannel);pwm.begin();pwm.setOscillatorFrequency(27000000);pwm.setPWMFreq(50);delay(50);
  clamp_neutral(neutralPulse1);clamp_neutral(neutralPulse2);clamp_neutral(neutralPulse3);clamp_neutral(neutralPulse4);
  init_motors();delay(150);Serial.println("ready");
}

void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r') continue;
    if(c=='r' || c=='R'){ start_move_all(+1,degrees_per_step); Serial.println("cmd: r -> all +"); }
    else if(c=='l' || c=='L'){ start_move_all(-1,degrees_per_step); Serial.println("cmd: l -> all -"); }
    else if(c=='s' || c=='0'){ cancel_all_and_center(); }
    // preserve individual stop keys if you want:
    else if(c=='9'){ start_move_idx(0,0,0); }
    else if(c=='8'){ start_move_idx(1,0,0); }
    else if(c=='7'){ start_move_idx(2,0,0); }
    else if(c=='6'){ start_move_idx(3,0,0); }
  }
  for(int i=0;i<4;i++) process_motor(motors[i]);
  delay(2);
}
