#ifndef CONTROLLER_CARTESIAN_H
#define CONTROLLER_CARTESIAN_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

class Controller_Cartesian: public Controller
{

public:
  Controller_Cartesian(): Controller() {};

  float f_attraction(float u);
  float f_repulsion(float u);
  float f_extra(float u);
  float get_attraction_velocity(float u);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_CARTESIAN_H*/