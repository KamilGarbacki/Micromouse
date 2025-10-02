//
// Created by kamil on 24.08.2025.
//

#include "maze_manager.h"

namespace micromouse {

MazeManager::MazeManager(const uint8_t rows, const uint8_t cols)
    : sizeX(cols), sizeY(rows) {
  this->maze = new Cell*[sizeY];  // rows
  for (int i = 0; i < sizeY; ++i) {
    this->maze[i] = new Cell[sizeX];  // cols
    for (int j = 0; j < sizeX; j++) {
      this->maze[i][j] = Cell(i, j, -1);
    }
  }

  goal = Cell();
}

MazeManager::~MazeManager() {
  for (int i = 0; i < sizeY; i++) {
    delete[] maze[i];
  }
  delete[] maze;
}

bool MazeManager::isValid(const int y, const int x) const {
  return (x >= 0 && x < sizeX && y >= 0 && y < sizeY);
}

void MazeManager::floodFill() {
  Serial.println("FloodFill 0");
  queue.push(&maze[goal.getY()][goal.getX()]);

  Serial.println("FloodFill 0.5");

  maze[goal.getY()][goal.getX()].setVal(0);

  Serial.println("FloodFill 1");

  Cell* c;
  Serial.println(queue.is_empty());
  while (!queue.is_empty()) {
    Serial.println("Loop");
     c = queue.pop_front();
    const int val = c->getVal();

    // down
    if (c->getY() + 1 < sizeY &&
        maze[c->getY() + 1][c->getX()].getVal() == -1 &&
        maze[c->getY() + 1][c->getX()].getAccUp()) {
      maze[c->getY() + 1][c->getX()].setVal(val + 1);
      queue.push(&maze[c->getY() + 1][c->getX()]);

      Serial.println("FloodFill 2");
    }

    // up
    if (c->getY() - 1 >= 0 && maze[c->getY() - 1][c->getX()].getVal() == -1 &&
        maze[c->getY() - 1][c->getX()].getAccDown()) {
      maze[c->getY() - 1][c->getX()].setVal(val + 1);
      queue.push(&maze[c->getY() - 1][c->getX()]);

      Serial.println("FloodFill 3");
    }

    // right
    if (c->getX() + 1 < sizeX &&
        maze[c->getY()][c->getX() + 1].getVal() == -1 &&
        maze[c->getY()][c->getX() + 1].getAccLeft()) {
      maze[c->getY()][c->getX() + 1].setVal(val + 1);
      queue.push(&maze[c->getY()][c->getX() + 1]);

      Serial.println("FloodFill 4");
    }

    // left
    if (c->getX() - 1 >= 0 && maze[c->getY()][c->getX() - 1].getVal() == -1 &&
        maze[c->getY()][c->getX() - 1].getAccRight()) {
      maze[c->getY()][c->getX() - 1].setVal(val + 1);
      queue.push(&maze[c->getY()][c->getX() - 1]);

      Serial.println("FloodFill 5");
    }
  }
  // delete c;
}

void MazeManager::setWall(const uint8_t y, const uint8_t x, const Direction dir) const {
  switch (dir) {
    case Direction::Down:
      if (y + 1 < sizeY) {
        maze[y + 1][x].setAccTop(false);
        maze[y][x].setAccDown(false);
      }
      break;

    case Direction::Up:
      if (y - 1 >= 0) {
        maze[y - 1][x].setAccDown(false);
        maze[y][x].setAccTop(false);
      }
      break;

    case Direction::Right:
      if (x + 1 < sizeX) {
        maze[y][x + 1].setAccLeft(false);
        maze[y][x].setAccRight(false);
      }
      break;

    case Direction::Left:
      if (x - 1 >= 0) {
        maze[y][x - 1].setAccRight(false);
        maze[y][x].setAccLeft(false);
      }
      break;
  }
}

void MazeManager::reset_maze() const {
  for (int y = 0; y < sizeY; y++) {
    for (int x = 0; x < sizeX; x++) {
      maze[y][x].setVal(-1);
    }
  }
  maze[goal.getY()][goal.getX()].setVal(0);
}

Direction MazeManager::getBestNeighbor(const uint8_t y, const uint8_t x) const {
  // const Cell& c = maze[y][x];
  // Map<Direction, int> dirValueMap;
  //
  // Serial.println("Start the finding of best neighbour");
  //
  // // Collect accessible neighbours
  // if (y + 1 < sizeY && c.getAccDown()) {
  //   Serial.println("if 1");
  //   dirValueMap.add(Direction::Down, maze[y + 1][x].getVal());
  //   Serial.println("after if 1");
  // }
  // if (y - 1 >= 0 && c.getAccUp()) {
  //   dirValueMap.add(Direction::Up, maze[y - 1][x].getVal());
  //   Serial.println("if 2");
  // }
  // if (x + 1 < sizeX && c.getAccRight()) {
  //   dirValueMap.add(Direction::Right, maze[y][x + 1].getVal());
  //   Serial.println("if 3");
  // }
  // if (x - 1 >= 0 && c.getAccLeft()) {
  //   dirValueMap.add(Direction::Left, maze[y][x - 1].getVal());
  //   Serial.println("if 4");
  // }
  //
  // Serial.println("After ifs");
  //
  // if (dirValueMap.size() == 0) {
  //   // No accessible neighbours
  //   return Direction::Up; // fallback
  // }
  //
  // Serial.println("check if size == 0");
  //
  // // Find the direction with the smallest value
  // Direction bestDir = dirValueMap.keys()[0];
  // int bestVal = dirValueMap.get(bestDir);
  //
  // Serial.println("get best val form value map");
  //
  // for (int i = 1; i < dirValueMap.size(); i++) {
  //   Serial.println("iterate through the map");
  //   const Direction dir = dirValueMap.keys()[i];
  //   const int val = dirValueMap.get(dir);
  //   if (val < bestVal) {
  //     Serial.println("in last if");
  //     bestVal = val;
  //     bestDir = dir;
  //   }
  // }
  //
  // return bestDir;

  const Cell c = maze[y][x];
  const int currVal = c.getVal();

  if (y + 1 < sizeY && c.getAccDown()) {
    if (maze[y + 1][x].getVal() < currVal) {
      return Direction::Down;
    }
  }
  if (y - 1 >= 0 && c.getAccUp()) {
    if (maze[y - 1][x].getVal() < currVal) {
      return Direction::Up;
    }
  }
  if (x + 1 < sizeX && c.getAccRight()) {
    if (maze[y][x + 1].getVal() < currVal) {
      return Direction::Right;
    }
  }
  if (x - 1 >= 0 && c.getAccLeft()) {
    if (maze[y][x - 1].getVal() < currVal) {
      return Direction::Left;
    }
  }

  return Direction::Up;
}

} // micromouse