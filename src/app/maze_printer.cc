#include "app/maze_printer.h"

#include "hal/log.h"
#include "navigation/maze_render.h"

namespace micromouse {

namespace {

void toSerial(const char c, void* /*context*/) {
  if (c == '\n') {
    hal::log::println();
  } else {
    hal::log::print(c);
  }
}

}  // namespace

void printMaze(const Maze& maze, const FloodFill& flood, const Cell robot) {
  renderMaze(maze, flood, robot, toSerial, nullptr);
}

}  // namespace micromouse
