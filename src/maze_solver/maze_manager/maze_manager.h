//
// Created by kamil on 24.08.2025.
//

#ifndef MAZE_MANAGER_H
#define MAZE_MANAGER_H
#include <stdint.h>
#include <Arduino.h>
#include "Map.h"
#include "maze_solver/cell/cell.h"
#include "direction/Direction.h"
#include "maze_solver/queue/Queue.h"

namespace micromouse {

class MazeManager {
  Cell** maze;
  uint8_t sizeX;
  uint8_t sizeY;

  Cell goal;

  util::Queue<Cell*> queue;


  bool isValid(int y, int x) const;

public:
  MazeManager(uint8_t rows, uint8_t cols);
  ~MazeManager();

  void setGoal(const uint8_t y, const uint8_t x) { goal = maze[y][x]; }
  Cell getGoal() const { return goal; }
  Cell getCell(const uint8_t y, const uint8_t x) const { return maze[y][x]; }

  void floodFill();
  void setWall(uint8_t x, uint8_t y, Direction dir) const;
  Direction getBestNeighbor(uint8_t y, uint8_t x) const;
  void reset_maze() const;

  void printMatrix(int xM, int yM) {
    int rows = sizeY;
    if (rows == 0) return;
    int cols = sizeX;

    for (int y = 0; y < rows; y++) {
      for (int x = 0; x < cols; x++) {
        // Print cell value
        if (xM == x && yM == y) {
          Serial.print("X");
        }
        else {
          Serial.print(maze[y][x].getVal());
        }


        // Check if there's a wall between this cell and the right one
        if (x < cols - 1) {
          if (!maze[y][x].getAccRight() || !maze[y][x + 1].getAccLeft()) {
            Serial.print("|");
          } else {
            Serial.print(" ");
          }
        }
      }
      Serial.println("");
    }
  }
};

} // micromouse

#endif //MAZE_MANAGER_H
