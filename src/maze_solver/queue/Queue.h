//
// Created by kamil on 23.08.2025.
//

#ifndef QUEUE_H
#define QUEUE_H
#include "Node.h"

namespace util {

template<typename T>
class Queue {
  Node<T>* tail = nullptr;
  Node<T>* head = nullptr;
  int size = 0;
public:
  ~Queue(); // destructor to clean up
  void push(T i);
  T pop_front();
  bool is_empty();
};

template<typename T>
Queue<T>::~Queue() {
  while (!is_empty()) {
    pop_front(); // safely frees all nodes
  }
}

template<typename T>
void Queue<T>::push(T i) {
  size++;

  Node<T>* x = new Node<T>{ i, nullptr };

  if (tail == nullptr) {
    tail = x;
    head = x;
  } else {
    tail->next = x;
    tail = x;
  }
}

template<typename T>
T Queue<T>::pop_front() {
  if (head != nullptr) {
    size--;

    Node<T>* temp = head;
    head = head->next;

    if (size == 0) {
      head = nullptr;
      tail = nullptr;
    }

    T val = temp->data;  // save before delete
    delete temp;         // free memory ✅
    return val;
  }
  return nullptr; // ⚠️ risky if T is not a pointer, but preserving your behavior
}

template<typename T>
bool Queue<T>::is_empty() {
  return head == nullptr;
}

// template<typename T>
//     class Queue {
//         Node<T> *firstNode;
//         long size = 0;
//
//         Node<T> *getNode(long index) const {
//             if (index < 0)
//                 index = size + index;
//
//             if (index > size - 1 || index < 0)
//                 return nullptr;
//
//             Node<T> *tempNode = firstNode;
//             if ((float) index < (float) size / 2)
//                 for (int i = 0; i != index; i++)
//                     tempNode = tempNode->nextNode;
//             else {
//                 index = size - index;
//
//                 for (int i = 0; i != index; i++)
//                     tempNode = tempNode->prevNode;
//             }
//
//             return tempNode;
//         }
//
//         void push(const long index, const T &obj) {
//             if (index == size) {
//                 push(obj);
//                 return;
//             }
//
//             if (index > size - 1)
//                 return;
//
//             auto currNode = getNode(index);
//             auto n = new Node<T>(currNode->prevNode, currNode, obj);
//
//             currNode->prevNode->nextNode = n;
//             currNode->prevNode = n;
//
//             if (index == 0)
//                 firstNode = n;
//
//             size++;
//         }
//
//     public:
//
//         void push(const T &obj) {
//             auto n = new Node<T>(obj);
//
//             if (firstNode == nullptr) {
//                 n->prevNode = n;
//                 firstNode = n;
//             }
//
//             Node<T> *tempLastNode = firstNode->prevNode;
//
//             n->prevNode = tempLastNode;
//             n->nextNode = firstNode;
//             tempLastNode->nextNode = n;
//             firstNode->prevNode = n;
//
//             size++;
//         }
//
//         T pop() {
//             if (size == 0)
//                 return nullptr;
//
//             T lastObj = firstNode->obj;
//             remove(0);
//
//             return lastObj;
//         }
//
//         [[nodiscard]] long getSize() const {
//             return size;
//         }
//
//         void reverse() {
//             if (size == 0)
//                 return;
//
//             auto tempFirstNode = firstNode;
//             auto currNode = tempFirstNode->prevNode;
//
//             while (currNode != tempFirstNode) {
//                 auto tempNode = currNode->nextNode;
//                 currNode->nextNode = currNode->prevNode;
//                 currNode->prevNode = tempNode;
//
//                 currNode = currNode->nextNode;
//             }
//
//             firstNode = firstNode->prevNode;
//         }
//
//         void remove(const long index) {
//             if (size == 0)
//                 return;
//
//             auto node = getNode(index);
//             auto newNext = node->nextNode;
//
//             if (index == 0)
//                 firstNode = newNext;
//
//             if (size == 1) {
//                 firstNode = nullptr;
//             } else {
//                 node->nextNode->prevNode = node->prevNode;
//                 node->prevNode->nextNode = newNext;
//             }
//
//             delete node;
//
//             size--;
//         }
//     };


} // util

#endif //QUEUE_H
