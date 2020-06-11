#ifndef FORAGE_H
#define FORAGE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

#define COMMAND_LOCAL 1 // Local frame

class forage: public Controller
{
  std::vector<float> motion_p; // Probability of motion
  uint timer; // Timer measuring how long a robot has been moving
  bool explore;
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
  bool holds_food, choose;
  float state;
public:
  forage();
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*FORAGE_H*/
