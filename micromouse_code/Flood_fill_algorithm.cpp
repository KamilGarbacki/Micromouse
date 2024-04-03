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
Cell::Cell(int a, int b, int x){
  y = a;
  x = b;
  val = x;
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

void flood_fill(Cell maze[4][6], Queue q, int size_y, int size_x) {
    while(!q.is_empty()){
        Cell c = q.pop_front();
        int val = c.val;
        //down
        if(c.get_y()+1 < size_y && maze[c.get_y() + 1][c.get_x()].val == -1 && maze[c.get_y() + 1][c.get_x()].acc_top){
            maze[c.get_y() + 1][c.get_x()].val = val + 1;
            q.push(maze[c.get_y() + 1][c.get_x()]);
        }
        //up
        if(c.get_y()-1 >= 0 && maze[c.get_y() - 1][c.get_x()].val == -1 && maze[c.get_y() - 1][c.get_x()].acc_down){
            maze[c.get_y() - 1][c.get_x()].val = val + 1;
            q.push(maze[c.get_y() - 1][c.get_x()]);
        }
        //right
        if(c.get_x()+1 < size_x && maze[c.get_y()][c.get_x() + 1].val == -1 && maze[c.get_y()][c.get_x() + 1].acc_left){
            maze[c.get_y()][c.get_x() + 1].val = val + 1;
            q.push(maze[c.get_y()][c.get_x() + 1]);
        }
        //left
        if(c.get_x()-1 >= 0 && maze[c.get_y()][c.get_x() - 1].val == -1 && maze[c.get_y()][c.get_x() - 1].acc_right) {
            maze[c.get_y()][c.get_x() - 1].val = val + 1;
            q.push(maze[c.get_y()][c.get_x() - 1]);
        }

    }
}

