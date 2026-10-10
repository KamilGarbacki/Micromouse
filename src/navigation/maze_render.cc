#include "navigation/maze_render.h"

namespace micromouse {

namespace {

constexpr uint8_t kCellWidth = 3;
constexpr uint8_t kDecimalBase = 10;

/// Emits characters one at a time: string literals would be placed in RAM
/// on the AVR, single characters are immediates.
class Writer {
 public:
  Writer(const CharSink sink, void* const context)
      : sink_(sink), context_(context) {}
  void put(const char c) const { sink_(c, context_); }
  void repeat(const char c, const uint8_t n) const {
    for (uint8_t i = 0; i < n; ++i) put(c);
  }

 private:
  CharSink sink_;
  void* context_;
};

void renderHorizontal(const Writer& out, const Maze& maze, const uint8_t row,
                      const Direction side) {
  for (uint8_t col = 0; col < maze.cols(); ++col) {
    const Cell cell{row, col};
    out.put('+');
    if (maze.hasWall(cell, side)) {
      out.repeat('-', kCellWidth);
    } else if (maze.isKnown(cell, side)) {
      out.repeat(' ', kCellWidth);
    } else {
      out.put(' ');
      out.put('.');
      out.put(' ');
    }
  }
  out.put('+');
  out.put('\n');
}

void renderVertical(const Writer& out, const Maze& maze, const Cell cell,
                    const Direction side) {
  if (maze.hasWall(cell, side)) {
    out.put('|');
  } else if (maze.isKnown(cell, side)) {
    out.put(' ');
  } else {
    out.put(':');
  }
}

void renderContent(const Writer& out, const Distance d, const bool isRobot) {
  if (isRobot || d == FloodFill::kUnreached) {
    out.put(' ');
    out.put(isRobot ? '@' : '-');
    out.put(' ');
    return;
  }
  char digits[kCellWidth];
  uint8_t count = 0;
  Distance value = d;
  do {
    digits[count++] = static_cast<char>('0' + value % kDecimalBase);
    value = static_cast<Distance>(value / kDecimalBase);
  } while (value > 0 && count < kCellWidth);
  out.repeat(' ', static_cast<uint8_t>(kCellWidth - count));
  while (count > 0) out.put(digits[--count]);
}

}  // namespace

void renderMaze(const Maze& maze, const FloodFill& flood, const Cell robot,
                const CharSink sink, void* const context) {
  if (sink == nullptr || maze.rows() == 0) return;
  const Writer out(sink, context);
  for (uint8_t row = 0; row < maze.rows(); ++row) {
    renderHorizontal(out, maze, row, Direction::kUp);
    for (uint8_t col = 0; col < maze.cols(); ++col) {
      const Cell cell{row, col};
      renderVertical(out, maze, cell, Direction::kLeft);
      renderContent(out, flood.distance(cell), cell == robot);
    }
    renderVertical(out, maze,
                   Cell{row, static_cast<uint8_t>(maze.cols() - 1)},
                   Direction::kRight);
    out.put('\n');
  }
  renderHorizontal(out, maze, static_cast<uint8_t>(maze.rows() - 1),
                   Direction::kDown);
}

}  // namespace micromouse
