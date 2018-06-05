#ifndef CONTROLLER_KEEP_AGGREGATE_H
#define CONTROLLER_KEEP_AGGREGATE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

#include <map>
#include <fstream>
#include "terminalinfo.h"
#include <sstream>
#include <random>
#include <iterator>
#include "auxiliary.h"
#include "template_calculator.h"

using namespace std;

class Controller_Keep_Aggregate: public Controller
{
  // Map of state-space index to possible action space indexes.
  std::map<int, vector<int>> state_action_matrix;
  OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.

  Template_Calculator t;
  int moving_timer;
  int selected_action;

  float _v_adj = 0.5; // Adjustment velocity
  int motion_dir = 0; // Use 0 for random or 1-8 to specific a direction.
  vector<float> beta_des;

public:
  Controller_Keep_Aggregate();
  ~Controller_Keep_Aggregate(){};
  
  void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);
  void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);
  void actionmotion(const int selected_action, float &v_x, float &v_y);

  float f_attraction(float u, float b);
  
  float get_attraction_velocity(float u, float b_eq);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_KEEP_AGGREGATE_H*/