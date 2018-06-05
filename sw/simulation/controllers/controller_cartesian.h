#ifndef CONTROLLER_CARTESIAN_H
#define CONTROLLER_CARTESIAN_H

#include "controller.h"

using namespace std;

class Controller_Cartesian: public Controller
{
  OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.

public:
  Controller_Cartesian(): Controller() {};

  float f_attraction(float u);
  
  float get_attraction_velocity(float u);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_CARTESIAN_H*/