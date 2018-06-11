#ifndef CONTROLLER_KEEP_AGGREGATE_H
#define CONTROLLER_KEEP_AGGREGATE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "auxiliary.h"
#include "template_calculator.h"
#include "controller_lattice_basic.h"

using namespace std;

class Controller_Keep_Aggregate : public Controller_Lattice_Basic
{
  // Map of state-space index to possible action space indexes.
  std::map<int, vector<int>> state_action_matrix;

  int moving_timer;
  int selected_action;

  int motion_dir = 0; // Use 0 for random or 1-8 to specific a direction.
  vector<float> beta_des;

public:
  Controller_Keep_Aggregate();
  ~Controller_Keep_Aggregate() {};
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_KEEP_AGGREGATE_H*/