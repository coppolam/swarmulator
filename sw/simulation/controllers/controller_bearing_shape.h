#ifndef CONTROLLER_BEARING_SHAPE_H
#define CONTROLLER_BEARING_SHAPE_H

#include "controller.h"

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "auxiliary.h"
#include "template_calculator.h"

using namespace std;

class Controller_Bearing_Shape: public Controller
{
  OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.
  Template_Calculator t;
  vector<float> beta_des;

  // TODO THIS DOESN'T WORK BECAUSE NEIGHBORS ARE NOT GLOBALLY UPDATED!
  int moving_timer;
  int selected_action;

  float _v_adj = 10;

public:
  Controller_Bearing_Shape();
  ~Controller_Bearing_Shape(){};
  
  void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);
  void actionmotion(const int selected_action, float &v_x, float &v_y);
  void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);

  float f_attraction(float u, float b);
  
  float get_attraction_velocity(float u, float b_eq);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);

};

#endif /*CONTROLLER_BEARING_SHAPE_H*/