#ifndef PFSM_EXPLORATION_H
#define PFSM_EXPLORATION_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "template_calculator.h"
#define COMMAND_LOCAL 1  // Local frame

class pfsm_exploration: public Controller
{
  Template_Calculator t;
  uint moving_timer;
  int selected_action;
  uint timelim;
  float vmean;
  std::vector<std::vector<float>> policy;
  uint st; // current state
  float vx_ref, vy_ref;
public:
  pfsm_exploration();
  void action_motion(const int &selected_action, float r, float t, float &v_x, float &v_y);
  void state_action_lookup(const int ID, uint state_index);
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*PFSM_EXPLORATION_H*/
