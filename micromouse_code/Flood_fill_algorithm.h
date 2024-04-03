#ifndef Flood_fill_algorithm.h
#define Flood_fill_algorithm.h

#include "Arduino.h"

class Cell{
private:
    int y;
    int x;
public:
    int val;
    bool acc_top;
    bool acc_down;
    bool acc_right;
    bool acc_left;

    Cell();
    Cell(int a, int b);
    Cell(int a, int b, int x);

    int get_x()const;
    int get_y()const;
};

class Node {
public:
    Cell data;
    Node* next;
};

class Queue {
    Node* tail = nullptr;
    Node* head = nullptr;
    int size = 0;
public:
    void push(Cell i);
    Cell pop_front();
    bool is_empty();
};

void flood_fill(Cell maze[4][6], Queue q, int size_y, int size_x);

void travel(Cell maze[4][6], int size_y, int size_x, Cell curr, int mice_dir);

#endif