#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

// #include <stdio.h>
// #include <iostream>
#include <stdint.h>
#include <vector>
using namespace std;

class Environment
{
  vector<vector<float>> walls;
public:
  Environment();
  ~Environment() {};
  void define(void);
  void add(void);
  bool sensor(uint8_t ID, vector<float> s_n, vector<float> s);
  void animate(void);
};

#endif /*ENVIRONMENT_H*/
