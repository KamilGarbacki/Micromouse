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
    Cell(int a, int b, int value);

    int get_x()const;
    int get_y()const;
};

template<typename T>
class Node {
public:
    T data;
    Node* next;
};

template<typename T>
class Queue {
    Node<T>* tail = nullptr;
    Node<T>* head = nullptr;
    int size = 0;
public:
    void push(T i);
    T pop_front();
    bool is_empty();
};


template<typename T> void Queue<T>::push(T i){
  size++;

  Node<T>* x = new Node<T>({ i, nullptr });

  if (tail == nullptr) {
    tail = x;
    head = x;
    }
  else {
      tail->next = x;
      tail = x;
  }
}

template<typename T> T Queue<T>::pop_front(){
  if (head != nullptr){
    size--;

    Node<T>* temp = head;
    head = head->next;

    if(size == 0){
      head = nullptr;
      tail = nullptr;
    }

    return(temp->data);
  }
}

template<typename T> bool Queue<T>::is_empty(){
  if (head == nullptr)
    return true;
  else
    return false;
}

void flood_fill(Cell t_maze[4][6], Queue<Cell> t_q, int size_y, int size_x);

void set_wall(Cell maze[4][6], Cell c1, int direction, int size_y, int size_x);

void reset_maze(Cell maze[4][6], int size_y, int size_x, Cell goal);

#endif