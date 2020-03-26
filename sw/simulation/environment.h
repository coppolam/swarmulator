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
  float limits(void);
  void add(float x0, float y0, float x1, float y1);
  bool sensor(uint8_t ID, vector<float> s_n, vector<float> s, float &slope_wall);
  void animate(void);
};

#endif /*ENVIRONMENT_H*/
