#ifndef FORAGE_H
#define FORAGE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;
#define COMMAND_LOCAL 1 // Local frame

class forage: public Controller
{
  vector<float> motion_p; // Probability of motion
  uint moving_timer, moving_timer_1; // Timer measuring how long a robot has been moving
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
  bool holds_food, choose;
public:
  forage();
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*FORAGE_H*/
