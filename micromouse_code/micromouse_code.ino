#include <NewPing.h>
 
//sonar 1, enc 2,3, sonar 4 motors 5,6,7,8, sonar 9,10,11,12   
 
const int E3 = 5; ///<Motor3 Speed
const int M3 = 8; ///<Motor3 Direction

const int E4 = 6; ///<Motor4 Speed
const int M4 = 7; ///<Motor4 Direction

const byte enc_l_pin = 2;
const byte enc_r_pin = 3;

volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;

#define TRIGGER_PIN_f 12
#define ECHO_PIN_f 11 //front

#define TRIGGER_PIN_l 10
#define ECHO_PIN_l 9 //left

#define TRIGGER_PIN_r 4 
#define ECHO_PIN_r 1 //right

#define RADAR_MAX_DISTANCE 200

NewPing sonar_f(TRIGGER_PIN_f, ECHO_PIN_f, RADAR_MAX_DISTANCE);
NewPing sonar_l(TRIGGER_PIN_l, ECHO_PIN_l, RADAR_MAX_DISTANCE);
NewPing sonar_r(TRIGGER_PIN_r, ECHO_PIN_r, RADAR_MAX_DISTANCE);

float move_dist = 100; //cm
int wall_dist = 10; //cm

char speed_l = 245;
char speed_r = 245;

char speed_offset = 5;

void count_l() {
  enc_l++;
}

void count_r() {
  enc_r++;
}

void M3_advance(char Speed){ ///<Motor3 Advance   left
 digitalWrite(M3,LOW);
 analogWrite(E3,Speed);
}
void M4_advance(char Speed){ ///<Motor4 Advance right
 digitalWrite(M4,LOW);
 analogWrite(E4,Speed);
}

void M3_back(char Speed){ ///<Motor3 Back off
 digitalWrite(M3,HIGH);
 analogWrite(E3,Speed);
}
void M4_back(char Speed){ ///<Motor4 Back off
 digitalWrite(M4,LOW);
 analogWrite(E4,Speed);
}

void stop(){
  analogWrite(E3,0);
  analogWrite(E4,0);
}

int dist_front(){
  return sonar_f.ping_cm();
}

int dist_left(){
  return sonar_l.ping_cm();
}

int dist_right(){
  return sonar_r.ping_cm();
}

void move(float dist){
  int pulses = (dist / (3.14f * 6.5f)) * 1900;

  int prev_l;
  int prev_r;

  unsigned int delta_l;
  unsigned int delta_r;

  unsigned int delta;

  enc_l = 0;
  enc_r = 0;

  while(enc_l <= pulses || enc_r <= pulses){
    delay(20);

    M3_advance(100);
    M4_advance(100);

    delta_l = enc_l - prev_l;
    delta_r = enc_r - prev_l;
    delta = delta_r - delta_r;
    if(delta_l > delta_r && delta >= 5)
      speed_l -= speed_offset;
    else if(delta_l < delta_r && delta >=5)
      speed_l += speed_offset;
  }

  stop()
}

void turn_r(){
  int pulses = (4 / (3.14f * 6.5f)) * 1900;

  int prev_l;
  int prev_r;

  unsigned int delta_l;
  unsigned int delta_r;

  unsigned int delta;

  enc_l = 0;
  enc_r = 0;

  while(enc_l <= pulses || enc_r <= pulses){
    delay(20);

    M3_advance(100);
    M4_back(100);

    delta_l = enc_l - prev_l;
    delta_r = enc_r - prev_l;
    delta = delta_r - delta_r;
    if(delta_l > delta_r && delta >= 5)
      speed_l -= speed_offset;
    else if(delta_l < delta_r && delta >=5)
      speed_l += speed_offset;
  }
  stop()
}

void turn_l(){
  int pulses = (4 / (3.14f * 6.5f)) * 1900;

  int prev_l;
  int prev_r;

  unsigned int delta_l;
  unsigned int delta_r;

  unsigned int delta;

  enc_l = 0;
  enc_r = 0;

  while(enc_l <= pulses || enc_r <= pulses){
    delay(20);

    M4_advance(100);
    M3_back(100);

    delta_l = enc_l - prev_l;
    delta_r = enc_r - prev_l;
    delta = delta_r - delta_r;
    if(delta_l > delta_r && delta >= 5)
      speed_l -= speed_offset;
    else if(delta_l < delta_r && delta >=5)
      speed_l += speed_offset;
  }
  stop()
}

void turn_around(){
  int pulses = (8 / (3.14f * 6.5f)) * 1900;

  int prev_l;
  int prev_r;

  unsigned int delta_l;
  unsigned int delta_r;

  unsigned int delta;

  enc_l = 0;
  enc_r = 0;

  while(enc_l <= pulses || enc_r <= pulses){
    delay(20);

    M3_advance(100);
    M4_back(100);

    delta_l = enc_l - prev_l;
    delta_r = enc_r - prev_l;
    delta = delta_r - delta_r;
    if(delta_l > delta_r && delta >= 5)
      speed_l -= speed_offset;
    else if(delta_l < delta_r && delta >=5)
      speed_l += speed_offset;
  }
  stop()
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(enc_l_pin), count_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), count_r, CHANGE);
}

void loop() {

}

