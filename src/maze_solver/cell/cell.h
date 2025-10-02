//
// Created by kamil on 23.08.2025.
//

#ifndef CELL_H
#define CELL_H

namespace micromouse {

class Cell {
  int y;
  int x;
  int val;
  bool acc_up;
  bool acc_down;
  bool acc_right;
  bool acc_left;

public:
  Cell() : y(0), x(0), val(-1),
           acc_up(true), acc_down(true), acc_right(true), acc_left(true) {}

  Cell(const int a, const int b) : y(a), x(b), val(-1),
                       acc_up(true), acc_down(true), acc_right(true), acc_left(true) {}

  Cell(const int a, const int b, const int value) : y(a), x(b), val(value),
                                  acc_up(true), acc_down(true), acc_right(true), acc_left(true) {}

  ~Cell() {}

  int getX() const { return x; }
  int getY() const { return y; }
  int getVal() const { return val; }
  bool getAccUp() const { return acc_up; }
  bool getAccDown() const { return acc_down; }
  bool getAccLeft() const { return acc_left; }
  bool getAccRight() const { return acc_right; }

  void setVal(const int v) { val = v; }
  void setAccTop(const bool v) { acc_up = v; }
  void setAccDown(const bool v) { acc_down = v; }
  void setAccLeft(const bool v) { acc_left = v; }
  void setAccRight(const bool v) { acc_right = v; }
};

}  // namespace micromouse

#endif  // CELL_H
