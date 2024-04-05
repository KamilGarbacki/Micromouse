#include "Flood_fill_algorithm.h"
#include "Arduino.h"

Cell::Cell(){}
Cell::Cell(int a, int b){
  y = a;
  x = b;
  val = -1;
  acc_top = true;
  acc_down = true;
  acc_right = true; 
  acc_left = true;
}
Cell::Cell(int a, int b, int value){
  y = a;
  x = b;
  val = value;

  acc_top = true;
  acc_down = true;
  acc_right = true; 
  acc_left = true;
}
int Cell::get_x()const{
  return x;
}
int Cell::get_y()const{
  return y;
}

void Queue::push(Cell i){
  size++;

  Node* x = new Node({ i, nullptr });

  if (tail == nullptr) {
    tail = x;
    head = x;
    }
  else {
      tail->next = x;
      tail = x;
  }
}

Cell Queue::pop_front(){
  if (head != nullptr){
    size--;

    Node* temp = head;
    head = head->next;

    if(size == 0){
      head = nullptr;
      tail = nullptr;
    }

    return(temp->data);
  }
}

bool Queue::is_empty(){
  if (head == nullptr)
    return true;
  else
    return false;
}

void flood_fill(Cell t_maze[4][6], Queue t_q, int size_y, int size_x) {
    while(!t_q.is_empty()){
        Cell c = t_q.pop_front();
        int val = c.val;
        //down
        if(c.get_y()+1 < size_y && t_maze[c.get_y() + 1][c.get_x()].val == -1 && t_maze[c.get_y() + 1][c.get_x()].acc_top){
            t_maze[c.get_y() + 1][c.get_x()].val = val + 1;
            t_q.push(t_maze[c.get_y() + 1][c.get_x()]);
        }
        //up
        if(c.get_y()-1 >= 0 && t_maze[c.get_y() - 1][c.get_x()].val == -1 && t_maze[c.get_y() - 1][c.get_x()].acc_down){
            t_maze[c.get_y() - 1][c.get_x()].val = val + 1;
            t_q.push(t_maze[c.get_y() - 1][c.get_x()]);
        }
        //right
        if(c.get_x()+1 < size_x && t_maze[c.get_y()][c.get_x() + 1].val == -1 && t_maze[c.get_y()][c.get_x() + 1].acc_left){
            t_maze[c.get_y()][c.get_x() + 1].val = val + 1;
            t_q.push(t_maze[c.get_y()][c.get_x() + 1]);
        }
        //left
        if(c.get_x()-1 >= 0 && t_maze[c.get_y()][c.get_x() - 1].val == -1 && t_maze[c.get_y()][c.get_x() - 1].acc_right) {
            t_maze[c.get_y()][c.get_x() - 1].val = val + 1;
            t_q.push(t_maze[c.get_y()][c.get_x() - 1]);
        }

    }
}

void set_wall(Cell maze[4][6], Cell c1, int direction, int size_y, int size_x){
    switch(direction) {
        case 0:
            if (c1.get_y() + 1 < size_y) {
                maze[c1.get_y() + 1][c1.get_x()].acc_top = false;
                maze[c1.get_y()][c1.get_x()].acc_down = false;
            }
            break;
        case 1:
            if (c1.get_y() - 1 >= 0) {
                maze[c1.get_y() - 1][c1.get_x()].acc_down = false;
                maze[c1.get_y()][c1.get_x()].acc_top = false;
            }
            break;
        case 2:
            if (c1.get_x() + 1 < size_x) {
                maze[c1.get_y()][c1.get_x() + 1].acc_left = false;
                maze[c1.get_y()][c1.get_x()].acc_right = false;
            }
            break;
        case 3:
            if (c1.get_x() - 1 >= 0) {
                maze[c1.get_y()][c1.get_x() - 1].acc_right = false;
                maze[c1.get_y()][c1.get_x()].acc_left = false;
            }
            break;
        default:
            break;
    }
}

void reset_maze(Cell maze[4][6], int size_y, int size_x, Cell goal){
    for (int i = 0; i < size_y; i++){
        for (int j = 0; j < size_x; j++) {
            maze[i][j].val = -1;
        }
    }
    maze[goal.get_y()][goal.get_x()].val = 0;
}



