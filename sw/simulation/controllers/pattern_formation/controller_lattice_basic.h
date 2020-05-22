#ifndef CONTROLLER_LATTICE_BASIC_H
#define CONTROLLER_LATTICE_BASIC_H

#include "controller.h"
#include "template_calculator.h"

using namespace std;

class controller_lattice_basic : public Controller
{
public:
  Template_Calculator t;
  vector<float> beta_des;
  float _v_adj = 1; // Adjustment velocity
  float d_safe = 0.9;

  controller_lattice_basic(): t(8) {};
  ~controller_lattice_basic() {};

  void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y);
  void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y);
  float f_attraction(const float &u, const float &b_eq);
  float get_attraction_velocity(const float &u, const float &b_eq);
  void actionmotion(const int &selected_action, float &v_x, float &v_y);
  bool check_motion(const vector<int> &state_ID);
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);
  void get_lattice_motion_all(const int &ID, const vector<int> &state_ID, const vector<uint> &closest, float &v_x,
                              float &v_y);
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y) = 0;
};

#endif /*CONTROLLER_LATTICE_BASIC_H*/
