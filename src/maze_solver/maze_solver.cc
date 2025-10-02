//
// Created by kamil on 20.08.2025.
//

#include "maze_solver.h"

namespace micromouse {

#define WALL_DISTANCE 20

Direction MazeSolver::getAbsoluteDir(Direction sensorDir) {
  int absDir = (static_cast<int>(mouseDir) + static_cast<int>(sensorDir)) % 4;
  return static_cast<Direction>(absDir);
}

int MazeSolver::getTurnDir(Direction targetDir) {
  return (static_cast<int>(targetDir) - static_cast<int>(mouseDir) + 4) % 4;
}

void MazeSolver::addSensor(const uint8_t triggerPin, const uint8_t echoPin,
                           const Direction sensorDir) {
  auto s = new UltrasonicDistSensor(triggerPin, echoPin, sensorDir);

  sensors.add(s);
}

void MazeSolver::updateCurrentCell(const Direction moveDir) {
  switch (moveDir) {
    case Direction::Up:
      currentCell = maze.getCell(currentCell.getY() - 1, currentCell.getX());
      break;
    case Direction::Down:
      currentCell = maze.getCell(currentCell.getY() + 1, currentCell.getX());
      break;
    case Direction::Left:
      currentCell = maze.getCell(currentCell.getY(), currentCell.getX() - 1);
      break;
    case Direction::Right:
      currentCell = maze.getCell(currentCell.getY(), currentCell.getX() + 1);
      break;
  }
}

void MazeSolver::solve() {
  if (currentCell.getY() == maze.getGoal().getY() &&
      currentCell.getX() == maze.getGoal().getX()) {
    return;
  }

  for (int i = 0; i < sensors.getSize(); i++) {
    UltrasonicDistSensor* s = sensors.get(i);

    const long d = s->readUltrasonicCM();

    if (d < WALL_DISTANCE) {
      maze.setWall(currentCell.getY(), currentCell.getX(),
                   getAbsoluteDir(s->getSensorDir()));
    }
    delay(150);
  }

  maze.reset_maze();

  maze.floodFill();

  maze.printMatrix(currentCell.getX(), currentCell.getY());

  const Direction nextMoveDir =
      maze.getBestNeighbor(currentCell.getY(), currentCell.getX());


  const int turn = getTurnDir(nextMoveDir);

  switch (turn) {
    case 1:
      engine.turn(true, 90);
      break;
    case 2:
      engine.turn(true, 180);
      break;
    case 3:
      engine.turn(false, 90);
      break;
    default:;
  }

  engine.driveToNextCell();


  delay(150);

  updateCurrentCell(nextMoveDir);

  mouseDir = nextMoveDir;
  solve();
}

}  // namespace micromouse