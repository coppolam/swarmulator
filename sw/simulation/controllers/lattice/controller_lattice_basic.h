#ifndef CONTROLLER_LATTICE_BASIC_H
#define CONTROLLER_LATTICE_BASIC_H

#include "controller.h"
#include "template_calculator.h"

using namespace std;

class Controller_Lattice_Basic : public Controller
{
public:
  OmniscientObserver o; // The omniscient observer is used to simulate sensing the other agents.
  Template_Calculator t;
  vector<float> beta_des;
  float _v_adj = 10; // Adjustment velocity
  float d_safe = 0.9;

  Controller_Lattice_Basic() {};
  ~Controller_Lattice_Basic() {};

  void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);
  void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);
  float f_attraction(const float &u, const float &b_eq);
  float get_attraction_velocity(const float &u, const float &b_eq);
  void actionmotion(const int &selected_action, float &v_x, float &v_y);
  bool check_motion(const vector<int> &state_ID);
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);
  void get_lattice_motion_all(const int &ID, const vector<int> &state_ID, const vector<int> &closest, float &v_x, float &v_y);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y) = 0;
};

#endif /*CONTROLLER_LATTICE_BASIC_H*/