#include "direction/Direction.h"
#include "engine/engine.h"
#include "maze_solver/maze_solver.h"
#include <Arduino.h>
// main.cpp

auto newEngine = micromouse::Engine();



void setup() {
  Serial.begin(9600);
  Serial.println();

  auto newEngine = micromouse::Engine();

  auto motorLeft = micromouse::Motor(5, 8, 2, micromouse::MotorMountSide::LEFT);
  auto motorRight = micromouse::Motor(11, 12, 3, micromouse::MotorMountSide::RIGHT);

  newEngine.addMotor(&motorLeft);
  newEngine.addMotor(&motorRight);

  auto mazeSolver = micromouse::MazeSolver(micromouse::Cell(4, 0, 0), micromouse::Direction::Up, 5, 4);

  mazeSolver.addSensor(6, 4, micromouse::Direction::Left);
  mazeSolver.addSensor(9, 7, micromouse::Direction::Up);
  mazeSolver.addSensor(13, 10, micromouse::Direction::Right);

  mazeSolver.setEngine(newEngine);
  mazeSolver.setGoal(0,3 );

  List<String> stepsTaken;
  mazeSolver.solve();
  Serial.println("Solved");
}

void loop() {

}
// main.cpp

