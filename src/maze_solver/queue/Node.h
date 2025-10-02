//
// Created by kamil on 23.08.2025.
//

#ifndef NODE_H
#define NODE_H

namespace util {

template<typename T>
class Node {
public:
  T data;
  Node<T>* next;
};

} // util

#endif //NODE_H
