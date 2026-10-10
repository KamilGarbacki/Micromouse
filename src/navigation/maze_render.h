// ASCII rendering of the discovered maze into a character sink (pure, so it
// can be tested on the host; the app sends the characters to Serial).
#pragma once

#include "navigation/flood_fill.h"
#include "navigation/maze.h"
#include "navigation/pose.h"

namespace micromouse {

/// Receives one character; '\n' ends a line.
using CharSink = void (*)(char c, void* context);

/// Renders the maze, row 0 at the top. Each cell is 3 characters wide and
/// shows its flood distance right-aligned, " - " if unreached, " @ " for the
/// robot. Walls: "---" / "|" known wall, "   " / " " known open,
/// " . " / ":" not observed yet. Corners are "+".
void renderMaze(const Maze& maze, const FloodFill& flood, Cell robot,
                CharSink sink, void* context);

}  // namespace micromouse
