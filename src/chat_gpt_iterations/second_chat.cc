// //
// // Created by kamil on 22.08.2025.
// //
// // Micromouse Maze Solver with PID Control
// // PlatformIO - Arduino Uno R3
// // All code is contained in this single file (main.cpp)
//
// /* === PIDController Class ===
//  * A simple PID controller for error correction using encoder counts.
//  */
// class PIDController {
//   public:
//     // PID gains
//     float Kp, Ki, Kd;
//     // state
//     float prevError;
//     float integral;
//     // constructor
//     PIDController(float p=0, float i=0, float d=0) : Kp(p), Ki(i), Kd(d), prevError(0), integral(0) {}
//     // Compute PID output given current error
//     float update(float error) {
//       integral += error;
//       float derivative = error - prevError;
//       prevError = error;
//       // PID formula
//       float output = Kp * error + Ki * integral + Kd * derivative;
//       return output;
//     }
//     // Reset the PID state (e.g., before a new motion)
//     void reset() {
//       prevError = 0;
//       integral = 0;
//     }
// };
//
// /* === Pin Definitions === */
// // Motor driver pins
// #define ENA 5    // PWM pin for left motor speed
// #define IN1 8    // Left motor direction 1
// #define IN2 9    // Left motor direction 2
//
// #define ENB 6    // PWM pin for right motor speed
// #define IN3 10   // Right motor direction 1
// #define IN4 11   // Right motor direction 2
//
// // Encoder pins (using interrupts)
// #define ENCODER_LEFT_PIN 2
// #define ENCODER_RIGHT_PIN 3
//
// // Ultrasonic sensor pins (trig and echo for front, left, and right)
// #define TRIG_FRONT 12
// #define ECHO_FRONT 13
// #define TRIG_LEFT 4
// #define ECHO_LEFT 7
// #define TRIG_RIGHT A0
// #define ECHO_RIGHT A1
//
// // Other constants
// #define WALL_THRESHOLD_CM 15   // Distance threshold to consider a wall (in cm)
// #define FORWARD_SPEED 150      // Base speed for straight movement (0-255)
// #define TURN_SPEED 130         // Base speed for turning
//
// // Encoder ticks needed for motions (example values, calibrate on your robot)
// #define ENCODER_TICKS_PER_CELL 1000   // Example value, calibrate to your robot
// #define ENCODER_TICKS_PER_TURN 300    // Example value, calibrate to your robot
//
// // Global variables for encoder counts (updated in interrupts)
// volatile long encoderCountLeft = 0;
// volatile long encoderCountRight = 0;
//
// // PID controllers for straight and turning
// // PID Gains (example values):
// // Start with Ki=0, Kd=0, increase Kp until wheels roughly aligned.
// // If a steady offset occurs, add a small Ki. Add Kd to smooth oscillations.
// PIDController pidStraight(0.6, 0.05, 0.1);  // example Kp, Ki, Kd for straight motion
// PIDController pidTurn(0.5, 0.01, 0.05);     // example Kp, Ki, Kd for 90° turn
//
// /* === Utility Functions === */
// // Set motor speeds; positive = forward, negative = backward
// void setMotorSpeeds(int speedLeft, int speedRight) {
//   // Left motor control
//   if (speedLeft > 0) {
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     analogWrite(ENA, speedLeft);
//   } else if (speedLeft < 0) {
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     analogWrite(ENA, -speedLeft);
//   } else {
//     analogWrite(ENA, 0);
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//   }
//   // Right motor control
//   if (speedRight > 0) {
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     analogWrite(ENB, speedRight);
//   } else if (speedRight < 0) {
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     analogWrite(ENB, -speedRight);
//   } else {
//     analogWrite(ENB, 0);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//   }
// }
//
// // Stop both motors
// void stopMotors() {
//   analogWrite(ENA, 0);
//   analogWrite(ENB, 0);
//   digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
//   digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
// }
//
// // Interrupt service routines for encoders (count pulses)
// void ISR_leftEncoder() {
//   // Increment or decrement based on motor direction
//   // If left motor is moving forward, count up; if backward, count down
//   if (digitalRead(IN1) == HIGH) {
//     encoderCountLeft++;
//   } else {
//     encoderCountLeft--;
//   }
// }
//
// void ISR_rightEncoder() {
//   if (digitalRead(IN3) == HIGH) {
//     encoderCountRight++;
//   } else {
//     encoderCountRight--;
//   }
// }
//
// // Read distance in cm from an ultrasonic sensor (HC-SR04 style)
// long readUltrasonicCM(int trigPin, int echoPin) {
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   // Measure echo pulse (timeout after 30ms)
//   long duration = pulseIn(echoPin, HIGH, 30000);
//   long distance = duration * 0.034 / 2; // speed of sound ~0.034 cm/us, half-roundtrip
//   return distance;
// }
//
// /* === Movement Helper Functions === */
//
// // Drive forward one cell (with PID to keep wheels in sync)
// void driveForwardCell() {
//   // Reset encoder counts and PID state
//   encoderCountLeft = 0;
//   encoderCountRight = 0;
//   pidStraight.reset();
//
//   // Drive forward loop: adjust until one cell distance traveled
//   while (abs(encoderCountLeft) < ENCODER_TICKS_PER_CELL ||
//          abs(encoderCountRight) < ENCODER_TICKS_PER_CELL) {
//     // Compute error = difference between left and right encoder counts
//     float error = (float)encoderCountLeft - (float)encoderCountRight;
//     // Get PID correction (positive means left ahead, so slow left / speed up right)
//     float correction = pidStraight.update(error);
//     // Calculate motor speeds with base speed and PID correction
//     int speedL = FORWARD_SPEED - (int)correction;
//     int speedR = FORWARD_SPEED + (int)correction;
//     // Constrain speeds to valid PWM range
//     speedL = constrain(speedL, 0, 255);
//     speedR = constrain(speedR, 0, 255);
//     // Set motors to forward with computed speeds
//     setMotorSpeeds(speedL, speedR);
//   }
//   // Stop at end of cell
//   stopMotors();
//   delay(50); // small pause to settle
// }
//
// // Turn 90 degrees (true = clockwise, false = counter-clockwise)
// void turn90Degrees(bool clockwise) {
//   // Reset encoder counts and PID state
//   encoderCountLeft = 0;
//   encoderCountRight = 0;
//   pidTurn.reset();
//
//   // Determine direction: one wheel forward, one backward
//   int dirL = clockwise ? 1 : -1;
//   int dirR = clockwise ? -1 : 1;
//
//   // Begin turn loop: adjust until ~90° rotation
//   while (abs(encoderCountLeft) < ENCODER_TICKS_PER_TURN ||
//          abs(encoderCountRight) < ENCODER_TICKS_PER_TURN) {
//     // Compute error between wheel counts
//     float error = (float)encoderCountLeft - (float)encoderCountRight;
//     // PID correction for turning
//     float correction = pidTurn.update(error);
//     // Base turn speeds (one forward, one backward)
//     int baseL = TURN_SPEED * dirL;
//     int baseR = TURN_SPEED * dirR;
//     // Apply correction: if error positive, reduce left speed, increase right speed, etc.
//     int speedL = baseL - (int)correction;
//     int speedR = baseR - (int)correction;
//     // Constrain to PWM limits
//     speedL = constrain(speedL, -255, 255);
//     speedR = constrain(speedR, -255, 255);
//     // Set motor speeds (handles direction from sign)
//     setMotorSpeeds(speedL, speedR);
//   }
//   // Stop after turn
//   stopMotors();
//   delay(50); // small pause to settle
// }
//
// /* === Flood-Fill / Maze Logic === */
// // This section is a placeholder. Integrate your flood-fill algorithm and map here.
// // For demonstration, we use a simple decision: go forward if no wall, else try right, etc.
//
// #define DIR_FORWARD 0
// #define DIR_RIGHT   1
// #define DIR_LEFT   -1
// #define DIR_BACK    2
//
// // Simple move decision: prefer forward, else right, else left, else back
// int decideNextMove(bool wallF, bool wallL, bool wallR) {
//   if (!wallF) {
//     return DIR_FORWARD;
//   } else if (!wallR) {
//     return DIR_RIGHT;
//   } else if (!wallL) {
//     return DIR_LEFT;
//   } else {
//     return DIR_BACK;
//   }
// }
//
// /* === Setup and Loop === */
// void setup() {
//   // Initialize serial (for debugging)
//   Serial.begin(9600);
//
//   // Motor pin initialization
//   pinMode(ENA, OUTPUT);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(ENB, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);
//
//   // Encoder pins
//   pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
//   pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), ISR_leftEncoder, RISING);
//   attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), ISR_rightEncoder, RISING);
//
//   // Ultrasonic sensor pins
//   pinMode(TRIG_FRONT, OUTPUT);
//   pinMode(ECHO_FRONT, INPUT);
//   pinMode(TRIG_LEFT, OUTPUT);
//   pinMode(ECHO_LEFT, INPUT);
//   pinMode(TRIG_RIGHT, OUTPUT);
//   pinMode(ECHO_RIGHT, INPUT);
//
//   // (Optional) Initialize maze data structure here
// }
//
// void loop() {
//   // 1. Read wall sensors
//   long distFront = readUltrasonicCM(TRIG_FRONT, ECHO_FRONT);
//   long distLeft  = readUltrasonicCM(TRIG_LEFT, ECHO_LEFT);
//   long distRight = readUltrasonicCM(TRIG_RIGHT, ECHO_RIGHT);
//   bool wallFront = (distFront > 0 && distFront < WALL_THRESHOLD_CM);
//   bool wallLeft  = (distLeft  > 0 && distLeft  < WALL_THRESHOLD_CM);
//   bool wallRight = (distRight > 0 && distRight < WALL_THRESHOLD_CM);
//
//   // (Optional) Update map with wallFront, wallLeft, wallRight here
//
//   // 2. Decide next move (using flood-fill or simple logic)
//   int moveDir = decideNextMove(wallFront, wallLeft, wallRight);
//
//   // 3. Execute move using PID-controlled motion
//   if (moveDir == DIR_FORWARD) {
//     driveForwardCell();
//   }
//   else if (moveDir == DIR_LEFT) {
//     turn90Degrees(false);  // turn counter-clockwise
//     driveForwardCell();
//   }
//   else if (moveDir == DIR_RIGHT) {
//     turn90Degrees(true);   // turn clockwise
//     driveForwardCell();
//   }
//   else if (moveDir == DIR_BACK) {
//     // U-turn (two 90° turns)
//     turn90Degrees(true);
//     turn90Degrees(true);
//     driveForwardCell();
//   }
//
//   // (Optional) Check for goal condition and stop if solved
//   // if (goalReached) { while(true) stopMotors(); }
// }