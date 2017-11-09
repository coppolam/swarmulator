#ifndef CONTROLLER_BEARING_SHAPE_H
#define CONTROLLER_BEARING_SHAPE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

class Controller_Bearing_Shape: public Controller
{

public:
  Controller_Bearing_Shape() : Controller() {};

  float f_attraction(float u, float b);
  float f_repulsion(float u);
  float f_extra(float u);
  float get_attraction_velocity(float u, float b_eq);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);

  float get_preferred_bearing(const vector<float> &bdes, const float v_b);
  void fill_template(vector<bool> &q, const float b_i, const float u, float dmax);
  void assess_situation(uint8_t ID, vector<bool> &q_old);
};

#endif /*CONTROLLER_BEARING_SHAPE_H*/