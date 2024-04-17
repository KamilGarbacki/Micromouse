#include <NewPing.h>
#include "Flood_fill_algorithm.h"

 
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
int wall_dist = 5; //cm

char speed_l = 245;
char speed_r = 245;

char speed_offset = 4;

  Queue<Cell> q;
  Queue<int> best_path;
  Cell goal(2, 4, 0);
  Cell start(3,0);
  Cell maze[4][6];

void count_l() {
  enc_l++;
}

void count_r() {
  enc_r++;
}

void M3_advance(char Speed){ ///<Motor3 Advance

 digitalWrite(M3,LOW);
 analogWrite(E3,Speed);
}

void M3_back(char Speed){ ///<Motor3 Back off

 digitalWrite(M3,HIGH);
 analogWrite(E3,Speed);
}


void M4_advance(char Speed){ ///<Motor4 Advance
 digitalWrite(M4,LOW);
 analogWrite(E4,Speed);
}


void M4_back(char Speed){ ///<Motor4 Back off
 digitalWrite(M4,HIGH);
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
    
    Serial.print("end lewy: ");
    Serial.println(enc_l);
    Serial.println("------");

    M3_advance(100);
    M4_advance(100);

    delta_l = enc_l - prev_l;
    delta_r = enc_r - prev_l;
    delta = delta_r - delta_r;
    if(delta_l > delta_r)
      speed_l -= speed_offset;
    else if(delta_l < delta_r)
      speed_l += speed_offset;
  }

  stop();
}

void turn_r(){
  int pulses = (12 / (3.14f * 6.5f)) * 1900;

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
  stop();
}

void turn_l(){
  int pulses = (12 / (3.14f * 6.5f)) * 1900;

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
  stop();
}

void turn_around(){
  int pulses = 2*(12 / (3.14f * 6.5f)) * 1900;

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
  stop();
}

void travel(Cell maze[4][6], int size_y, int size_x, Cell curr, int mice_dir){

    if(maze[curr.get_y()][curr.get_x()].val == 0){
        //std::cout << "end" << std::endl;
        return;
    }

    int sensor_dir;
    if(dist_front() > wall_dist){
      sensor_dir = mice_dir;
      set_wall(maze, curr, sensor_dir, size_y, size_x);
    }
    if(dist_right() > wall_dist){
      sensor_dir = (1 + mice_dir) % 4;
      set_wall(maze, curr, sensor_dir, size_y, size_x);
    }
    if(dist_left() > wall_dist){
      sensor_dir = (2 + mice_dir) % 4;
      set_wall(maze, curr, sensor_dir, size_y, size_x);
    }
    reset_maze(maze, size_y, size_x, goal);
    flood_fill(maze, q, size_y, size_x);
    
    Cell next_move;
    next_move.val = 999;
    int direction;

    //down
    if(curr.get_y()+1 < size_y && maze[curr.get_y() + 1][curr.get_x()].acc_top && maze[curr.get_y() + 1][curr.get_x()].val < next_move.val){
        next_move = maze[curr.get_y() + 1][curr.get_x()];
        direction = 2;
    }
    //up
    if(curr.get_y()-1 >= 0 && maze[curr.get_y() - 1][curr.get_x()].acc_down && maze[curr.get_y() - 1][curr.get_x()].val < next_move.val){
        next_move = maze[curr.get_y() - 1][curr.get_x()];
        direction = 0;
    }
    //right
    if(curr.get_x()+1 < size_x && maze[curr.get_y()][curr.get_x() + 1].acc_left && maze[curr.get_y()][curr.get_x() + 1].val < next_move.val){
        next_move = maze[curr.get_y()][curr.get_x() + 1];
        direction = 1;
    }
    //left
    if(curr.get_x()-1 >= 0 && maze[curr.get_y()][curr.get_x() - 1].acc_right && maze[curr.get_y()][curr.get_x() - 1].val < next_move.val){
        next_move = maze[curr.get_y()][curr.get_x() - 1];
        direction = 3;
    }

    if(mice_dir == direction){}
    else if(mice_dir-direction == 2 || mice_dir-direction == -2)
        turn_around();
    else if((mice_dir == direction+1) ||(direction == mice_dir+3))
        turn_l();
    else if((mice_dir+1 == direction) ||(direction=3 == mice_dir))
        turn_r();
    
    move(move_dist);
    mice_dir = direction;

    travel(maze, size_y, size_x, next_move, mice_dir);
}

void get_best_route(Cell maze[4][6], int size_y, int size_x, Cell curr, int mice_dir, Queue<int> opt_moves){

    if(maze[curr.get_y()][curr.get_x()].val == 0){
        //std::cout << "end" << std::endl;
        return;
    }

    flood_fill(maze, q, size_y, size_x);
    
    Cell next_move;
    next_move.val = 999;
    int direction;

    //down
    if(curr.get_y()+1 < size_y && maze[curr.get_y() + 1][curr.get_x()].acc_top && maze[curr.get_y() + 1][curr.get_x()].val < next_move.val){
        next_move = maze[curr.get_y() + 1][curr.get_x()];
        direction = 2;
    }
    //up
    if(curr.get_y()-1 >= 0 && maze[curr.get_y() - 1][curr.get_x()].acc_down && maze[curr.get_y() - 1][curr.get_x()].val < next_move.val){
        next_move = maze[curr.get_y() - 1][curr.get_x()];
        direction = 0;
    }
    //right
    if(curr.get_x()+1 < size_x && maze[curr.get_y()][curr.get_x() + 1].acc_left && maze[curr.get_y()][curr.get_x() + 1].val < next_move.val){
        next_move = maze[curr.get_y()][curr.get_x() + 1];
        direction = 1;
    }
    //left
    if(curr.get_x()-1 >= 0 && maze[curr.get_y()][curr.get_x() - 1].acc_right && maze[curr.get_y()][curr.get_x() - 1].val < next_move.val){
        next_move = maze[curr.get_y()][curr.get_x() - 1];
        direction = 3;
    }

  if(mice_dir != direction){
  if(mice_dir-direction == 2 || mice_dir-direction == -2)
    opt_moves.push(3);
  else if((mice_dir == direction+1) ||(direction == mice_dir+3))
    opt_moves.push(1);
  else if((mice_dir+1 == direction) ||(direction=3 == mice_dir))
    opt_moves.push(2);
  }
  opt_moves.push(0);
  
  mice_dir = direction;

  get_best_route(maze, size_y, size_x, next_move, mice_dir, opt_moves);
}

void setup() {
  Serial.println();
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(enc_l_pin), count_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_r_pin), count_r, CHANGE);

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);

  

//   for (int i = 0; i < 4; i++) {
//     for (int j = 0; j < 6; j++) {
//       maze[i][j] = Cell(i,j,-1);
//     }
//   }
//   Serial.println("---------------------");

//  maze[goal.get_y()][goal.get_x()].val = 0;

//   q.push(goal);
//   flood_fill(maze, q, 4, 6);
  
//   for (int i = 0; i < 4; i++) {
//     for (int j = 0; j < 6; j++) {
//       Serial.print(maze[i][j].val);
//     }
//     Serial.println();
//   }

//   travel(maze, 4, 6, start, 0);

//   delay(6000);

//   get_best_route(maze, 4, 6, start, 0, best_path);

//   while(!best_path.is_empty()){
//     int next_move = best_path.pop_front();

//     switch(next_move){
//       case 0:
//         move(move_dist);
//         break;
//       case 1:
//         turn_l();
//         break;
//       case 2:
//         turn_r();
//         break;
//       case 3:
//         turn_around();
//         break;
//     }

//   }
  
}

void loop() {
  move(25);
  delay(1500);
}

