// ASCII dump of the discovered maze over Serial.
#pragma once

#include "navigation/flood_fill.h"
#include "navigation/maze.h"
#include "navigation/pose.h"

namespace micromouse {

/// Prints the maze over Serial in the format of renderMaze()
/// (navigation/maze_render.h).
void printMaze(const Maze& maze, const FloodFill& flood, Cell robot);

}  // namespace micromouse
