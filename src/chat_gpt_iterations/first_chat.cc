// //
// // Created by kamil on 22.08.2025.
// //
// // ==== Configurable Parameters ====
// const int MAZE_SIZE    = 16;  // Maze width/height (max 16 for UNO)
// const int GOAL_X       = 7;   // Goal cell (example: (7,7) in a 16x16)
// const int GOAL_Y       = 7;
//
// // Motor driver pins (using DRI0039 shield channels M3 & M4)
// const int LEFT_PWM_PIN   = 5; // M3 speed (PWM)
// const int LEFT_DIR_PIN   = 8; // M3 direction
// const int RIGHT_PWM_PIN  = 6; // M4 speed (PWM)
// const int RIGHT_DIR_PIN  = 7; // M4 direction
//
// // Encoder pins (attach to interrupt-capable pins 2 and 3)
// const int ENC_LEFT_PIN  = 2;  // Left motor encoder (INT0)
// const int ENC_RIGHT_PIN = 3;  // Right motor encoder (INT1)
//
// // Ultrasonic sensor pins
// const int US_TRIG_FRONT = A0;
// const int US_ECHO_FRONT = A1;
// const int US_TRIG_LEFT  = A2;
// const int US_ECHO_LEFT  = A3;
// const int US_TRIG_RIGHT = 9;
// const int US_ECHO_RIGHT = 10;
//
// // Movement parameters (tune these by testing)
// const long PULSES_PER_CELL = 300;   // Encoder pulses for one cell distance
// const long PULSES_PER_TURN = 200;   // Pulses for ~90-degree in-place turn
// const int  MOTOR_SPEED     = 150;   // PWM speed for motors (0-255)
//
// // Distance threshold (cm) for detecting a wall
// const int OBSTACLE_DIST = 15;
//
// // Orientation: 0=North, 1=East, 2=South, 3=West
// int orientation = 0;
//
// // Current position in grid coordinates
// int posX = 0, posY = 0;
//
// // Encoder pulse counts (volatile for ISR updates)
// volatile long leftCount = 0;
// volatile long rightCount = 0;
//
// // Maze cell structure: walls and flood-fill distance
// struct Cell {
//     bool wall[4];    // wall[N], wall[E], wall[S], wall[W]
//     int dist;        // distance from this cell to goal
//     Cell() {
//         for(int i=0; i<4; i++) wall[i] = false;
//         dist = 10000;
//     }
// };
// Cell maze[MAZE_SIZE][MAZE_SIZE];  // The maze grid
//
// // ==== Interrupt Service Routines ====
// void leftEncoderISR()  { leftCount++; }
// void rightEncoderISR() { rightCount++; }
//
// // ==== Utility Functions ====
//
// // Drive motors: positive speed=forward, negative=backward.
// // Left motor forward = digitalWrite(LEFT_DIR_PIN, LOW) (per shield wiring).
// void driveMotors(int speedLeft, int speedRight) {
//     // Left motor
//     if (speedLeft >= 0) {
//         digitalWrite(LEFT_DIR_PIN, LOW);
//         analogWrite(LEFT_PWM_PIN, speedLeft);
//     } else {
//         digitalWrite(LEFT_DIR_PIN, HIGH);
//         analogWrite(LEFT_PWM_PIN, -speedLeft);
//     }
//     // Right motor (assuming reversed polarity: HIGH=forward)
//     if (speedRight >= 0) {
//         digitalWrite(RIGHT_DIR_PIN, HIGH);
//         analogWrite(RIGHT_PWM_PIN, speedRight);
//     } else {
//         digitalWrite(RIGHT_DIR_PIN, LOW);
//         analogWrite(RIGHT_PWM_PIN, -speedRight);
//     }
// }
//
// // Read ultrasonic distance (cm) with timeout (in µs).
// long readUltrasonicCM(int trigPin, int echoPin) {
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);
//     long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
//     if (duration == 0) return 1000; // no echo (too far or error)
//     return duration * 0.034 / 2;    // speed of sound ~0.034 cm/µs
// }
//
// // Convert a relative direction (−1=left,0=front,+1=right) to an absolute direction index 0-3.
// int absDir(int rel) {
//     // For example, if orientation=0 (North):
//     //   rel=0 (front) => 0 (North)
//     //   rel=+1 (right) => 1 (East)
//     //   rel= -1 (left) => 3 (West)
//     int abs = orientation + rel;
//     if (abs < 0) abs += 4;
//     return abs % 4;
// }
//
// // Mark a wall in cell (x,y) in direction dir (0=N,1=E,2=S,3=W).
// // Also mark the opposite wall in the neighboring cell.
// void setWall(int x, int y, int dir) {
//     if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;
//     maze[x][y].wall[dir] = true;
//     int nx = x, ny = y;
//     int odir = (dir + 2) % 4;
//     if      (dir == 0) ny--; // north neighbor
//     else if (dir == 1) nx++;
//     else if (dir == 2) ny++;
//     else if (dir == 3) nx--;
//     if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
//         maze[nx][ny].wall[odir] = true;
//     }
// }
//
// // Perform flood-fill (BFS) to compute distances from goal.
// void floodFill() {
//     // Reset distances to large number
//     for(int ix=0; ix<MAZE_SIZE; ix++) {
//         for(int iy=0; iy<MAZE_SIZE; iy++) {
//             maze[ix][iy].dist = 10000;
//         }
//     }
//     // Simple queue for BFS
//     struct Node { int x, y; };
//     Node queue[MAZE_SIZE*MAZE_SIZE];
//     int front=0, back=0;
//
//     // Start from goal cell
//     maze[GOAL_X][GOAL_Y].dist = 0;
//     queue[back++] = {GOAL_X, GOAL_Y};
//
//     // Directions dx,dy for N,E,S,W
//     int dx[4] = {0, 1, 0, -1};
//     int dy[4] = {-1, 0, 1, 0};
//
//     // BFS loop
//     while (front < back) {
//         Node cur = queue[front++];
//         int cx = cur.x, cy = cur.y;
//         int cd = maze[cx][cy].dist;
//         // Check 4 neighbors
//         for(int d=0; d<4; d++) {
//             int nx = cx + dx[d], ny = cy + dy[d];
//             // If neighbor is in bounds and no wall between cur and neighbor
//             if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE && !maze[cx][cy].wall[d]) {
//                 if (maze[nx][ny].dist > cd + 1) {
//                     maze[nx][ny].dist = cd + 1;
//                     queue[back++] = {nx, ny};
//                 }
//             }
//         }
//     }
// }
//
// // Choose next direction relative to current heading: -1=left, 0=forward, +1=right
// int chooseDirection() {
//     int bestRel = 0;
//     int bestDist = 10000;
//     // Check front (0), right (+1), and left (-1)
//     int rels[3] = {0, +1, -1};
//     for (int i = 0; i < 3; i++) {
//         int rel = rels[i];
//         int d = absDir(rel);
//         // Compute neighbor coordinates if moving in direction d
//         int nx = posX, ny = posY;
//         if (d == 0)      ny--;
//         else if (d == 1) nx++;
//         else if (d == 2) ny++;
//         else if (d == 3) nx--;
//         // Check bounds and wall
//         if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE && !maze[posX][posY].wall[d]) {
//             int dist = maze[nx][ny].dist;
//             if (dist < bestDist) {
//                 bestDist = dist;
//                 bestRel = rel;
//             }
//         }
//     }
//     return bestRel;
// }
//
// void setup() {
//     Serial.begin(9600);
//     // Initialize motor pins
//     pinMode(LEFT_DIR_PIN, OUTPUT);
//     pinMode(RIGHT_DIR_PIN, OUTPUT);
//     pinMode(LEFT_PWM_PIN, OUTPUT);
//     pinMode(RIGHT_PWM_PIN, OUTPUT);
//     // Initialize encoder pins and interrupts
//     pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
//     pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), leftEncoderISR, RISING);
//     attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), rightEncoderISR, RISING);
//     // Initialize ultrasonic sensor pins
//     pinMode(US_TRIG_FRONT, OUTPUT); pinMode(US_ECHO_FRONT, INPUT);
//     pinMode(US_TRIG_LEFT,  OUTPUT); pinMode(US_ECHO_LEFT,  INPUT);
//     pinMode(US_TRIG_RIGHT, OUTPUT); pinMode(US_ECHO_RIGHT, INPUT);
//     // Initialize maze (no walls known yet)
//     floodFill();
// }
//
// void loop() {
//     // Check if goal reached
//     if (posX == GOAL_X && posY == GOAL_Y) {
//         driveMotors(0,0);
//         Serial.println("Goal reached!");
//         while(1); // Stop
//     }
//
//     // --- 1. Sense walls with ultrasonic sensors ---
//     long distF = readUltrasonicCM(US_TRIG_FRONT, US_ECHO_FRONT);
//     delay(50);
//     long distL = readUltrasonicCM(US_TRIG_LEFT, US_ECHO_LEFT);
//     delay(50);
//     long distR = readUltrasonicCM(US_TRIG_RIGHT, US_ECHO_RIGHT);
//
//     // Interpret readings as walls based on orientation
//     if (distF < OBSTACLE_DIST) setWall(posX, posY, absDir(0));  // front
//     if (distL < OBSTACLE_DIST) setWall(posX, posY, absDir(-1)); // left
//     if (distR < OBSTACLE_DIST) setWall(posX, posY, absDir(+1)); // right
//
//     // --- 2. Update flood-fill distances ---
//     floodFill();
//
//     // --- 3. Decide next move ---
//     int turn = chooseDirection(); // -1=left, 0=front, +1=right
//
//     // --- 4. Execute turn if needed ---
//     if (turn == -1) {
//         // Turn left 90 degrees: left wheel backward, right forward
//         leftCount = rightCount = 0;
//         driveMotors(-MOTOR_SPEED, +MOTOR_SPEED);
//         while (abs(leftCount) < PULSES_PER_TURN && abs(rightCount) < PULSES_PER_TURN) {}
//         driveMotors(0,0);
//         orientation = (orientation + 3) % 4;
//     }
//     else if (turn == +1) {
//         // Turn right 90 degrees: left forward, right backward
//         leftCount = rightCount = 0;
//         driveMotors(+MOTOR_SPEED, -MOTOR_SPEED);
//         while (abs(leftCount) < PULSES_PER_TURN && abs(rightCount) < PULSES_PER_TURN) {}
//         driveMotors(0,0);
//         orientation = (orientation + 1) % 4;
//     }
//     // (If turn==0, no rotation, keep facing same way)
//
//     // --- 5. Move forward one cell ---
//     leftCount = rightCount = 0;
//     driveMotors(+MOTOR_SPEED, +MOTOR_SPEED);
//     while (leftCount < PULSES_PER_CELL && rightCount < PULSES_PER_CELL) {}
//     driveMotors(0,0);
//
//     // Update position (based on orientation)
//     if      (orientation == 0) posY--;
//     else if (orientation == 1) posX++;
//     else if (orientation == 2) posY++;
//     else if (orientation == 3) posX--;
//
//     Serial.print("Moved to (");
//     Serial.print(posX); Serial.print(",");
//     Serial.print(posY); Serial.println(")");
// }