//
// Created by kamil on 20.08.2025.
//

#ifndef MAZE_SOLVER_H
#define MAZE_SOLVER_H
#include <stdint.h>

#include <List.hpp>

#include "cell/cell.h"
#include "direction/Direction.h"
#include "engine/engine.h"
#include "maze_manager/maze_manager.h"
#include "sensor/distance_sensor/ultrasonic_dist_sensor/ultrasonic_dist_sensor.h"

namespace micromouse {

class MazeSolver {
  MazeManager maze;
  Cell currentCell; 
  Direction mouseDir;
  Engine engine;
  List<UltrasonicDistSensor*> sensors;

  Direction getAbsoluteDir(Direction sensorDir);
  int getTurnDir(Direction targetDir);
  void updateCurrentCell(Direction moveDir);
public:

  MazeSolver(const Cell& curr, const Direction dir, const uint8_t rows,
            const uint8_t cols)
    : maze(MazeManager(rows, cols)), currentCell(curr), mouseDir(dir) {}

  void setGoal(const uint8_t y, const uint8_t x) { maze.setGoal(y, x); }

  void setEngine(const Engine &e) {engine = e;}

  void addSensor(uint8_t triggerPin, uint8_t echoPin, Direction sensorDir);

  void solve();
};

}  // namespace micromouse

#endif  // MAZE_SOLVER_H
