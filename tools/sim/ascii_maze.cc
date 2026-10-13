#include "ascii_maze.h"

namespace sim {

namespace {

using micromouse::Cell;
using micromouse::Direction;
using micromouse::Maze;

std::vector<std::string> splitLines(const std::string& text) {
  std::vector<std::string> lines;
  std::string line;
  for (const char c : text) {
    if (c == '\n') {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      lines.push_back(line);
      line.clear();
    } else {
      line.push_back(c);
    }
  }
  if (!line.empty()) {
    if (line.back() == '\r') line.pop_back();
    lines.push_back(line);
  }
  // Drop trailing blank lines (a stray newline at the end of the file).
  while (!lines.empty() &&
        lines.back().find_first_not_of(" \t") == std::string::npos) {
    lines.pop_back();
  }
  return lines;
}

}  // namespace

bool parseMazeText(const std::string& text, ParsedMaze& out,
                   std::vector<std::string>& errors) {
  const std::vector<std::string> lines = splitLines(text);
  if (lines.size() < 3 || lines.size() % 2 == 0) {
    errors.push_back(
        "maze must have an odd number of lines >= 3: a horizontal wall line, "
        "then alternating content/horizontal lines (got " +
        std::to_string(lines.size()) + ")");
    return false;
  }

  const uint8_t rows = static_cast<uint8_t>((lines.size() - 1) / 2);
  const size_t lineLen = lines[0].size();
  if (lineLen < 5 || (lineLen - 1) % 4 != 0) {
    errors.push_back("line 1: width " + std::to_string(lineLen) +
                     " must be of the form 4*cols+1 (each cell is 3 chars "
                     "wide plus a '+' or '|' separator)");
    return false;
  }
  const uint8_t cols = static_cast<uint8_t>((lineLen - 1) / 4);

  for (size_t i = 0; i < lines.size(); ++i) {
    if (lines[i].size() != lineLen) {
      errors.push_back("line " + std::to_string(i + 1) + ": width " +
                       std::to_string(lines[i].size()) + ", expected " +
                       std::to_string(lineLen) +
                       " (every line must line up with line 1)");
    }
  }
  if (!errors.empty()) return false;

  if (rows > Maze::kMaxRows || cols > Maze::kMaxCols) {
    errors.push_back(
        "maze " + std::to_string(rows) + "x" + std::to_string(cols) +
        " exceeds the firmware's static capacity " +
        std::to_string(Maze::kMaxRows) + "x" + std::to_string(Maze::kMaxCols) +
        " (config::kMazeMaxRows / kMazeMaxCols)");
    return false;
  }

  out = ParsedMaze{};
  out.rows = rows;
  out.cols = cols;
  if (out.truth.reset(rows, cols, 1) != micromouse::Status::kOk) {
    errors.push_back("internal error: Maze::reset() rejected " +
                     std::to_string(rows) + "x" + std::to_string(cols));
    return false;
  }

  // Horizontal wall lines: index 0, 2, 4, ... (2*r is the line above row r,
  // and also the line below row r-1). Only the interior ones (0 < r < rows)
  // can change anything: the outer boundary is always a wall, regardless of
  // what the text shows there.
  for (uint8_t r = 1; r < rows; ++r) {
    const std::string& line = lines[2u * r];
    for (uint8_t c = 0; c < cols; ++c) {
      const std::string seg = line.substr(1 + c * 4, 3);
      if (seg.find('-') != std::string::npos) {
        out.truth.setWall(Cell{static_cast<uint8_t>(r - 1), c},
                          Direction::kDown, true);
      }
    }
  }

  // Content lines: index 1, 3, 5, ... Vertical separators at c*4 (c = 0
  // .. cols); c = 0 and c = cols are the outer boundary (always a wall).
  // Cell content is the 3 characters between separators; 'S'/'G' mark the
  // start / goal.
  for (uint8_t r = 0; r < rows; ++r) {
    const std::string& line = lines[2u * r + 1];
    for (uint8_t c = 1; c < cols; ++c) {
      if (line[c * 4] == '|') {
        out.truth.setWall(Cell{r, static_cast<uint8_t>(c - 1)},
                          Direction::kRight, true);
      }
    }
    for (uint8_t c = 0; c < cols; ++c) {
      const std::string content = line.substr(1 + c * 4, 3);
      for (const char ch : content) {
        if (ch == 'S' || ch == 's') {
          if (out.hasStart) errors.push_back("more than one 'S' (start) marker");
          out.hasStart = true;
          out.start = Cell{r, c};
        } else if (ch == 'G' || ch == 'g') {
          if (out.hasGoal) errors.push_back("more than one 'G' (goal) marker");
          out.hasGoal = true;
          out.goal = Cell{r, c};
        }
      }
    }
  }
  return errors.empty();
}

}  // namespace sim
