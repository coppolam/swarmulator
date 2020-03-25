#ifndef ARENA_H
#define ARENA_H

// #include <stdio.h>
// #include <iostream>
#include <stdint.h>
#include <vector>
using namespace std;

class Arena
{
  vector<vector<float>> walls;
public:
  Arena();
  ~Arena() {};
  void define(void);
  void add(void);
  bool sensor(uint8_t ID, vector<float> s_n, vector<float> s);
  void animate(void);
};

#endif /*ARENA_H*/
