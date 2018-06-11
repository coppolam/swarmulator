#include "controller_lattice_basic.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

float Controller_Lattice_Basic::f_attraction(const float &u, const float &b_eq)
{
  float w;
  // To do -- move general attraction equation to controller
  if (!(abs(b_eq - M_PI / 4.0) < 0.1 || abs(b_eq - (3 * M_PI / 4.0)) < 0.1)) {
    w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
  } else {
    w = log((sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)) / _kr - 1) / exp(-_ka * sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)))) / _ka;
  }

  return 1 / (1 + exp(-_ka * (u - w)));
}

// To do -- move general sum to controller
float Controller_Lattice_Basic::get_attraction_velocity(const float &u, const float &b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u);
}

void Controller_Lattice_Basic::attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}

void Controller_Lattice_Basic::latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}

void Controller_Lattice_Basic::actionmotion(const int &selected_action, float &v_x, float &v_y)
{
  float actionspace_y[8] = {0, sqrt(1), 1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1)};
  float actionspace_x[8] = {1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1), 0, sqrt(1)};
  v_x = _v_adj * actionspace_x[selected_action];
  v_y = _v_adj * actionspace_y[selected_action];
}

bool Controller_Lattice_Basic::check_motion(const vector<int> &state_ID)
{
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (o->see_if_moving(state_ID[i])) {
      // Somebody nearby is already moving
      canImove = false;
    }
  }
  return canImove;
}

void Controller_Lattice_Basic::get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y)
{
  float v_b, b_eq, v_r;
  v_b = wrapToPi_f(o->request_bearing(ID, state_ID));
  b_eq = t.get_preferred_bearing(beta_des, v_b);
  v_r = get_attraction_velocity(o->request_distance(ID, state_ID), b_eq);
  latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
}

void Controller_Lattice_Basic::get_lattice_motion_all(const int &ID, const vector<int> &state_ID, const vector<int> &closest, float &v_x, float &v_y)
{
  if (!state_ID.empty()) {
    if (o->request_distance(ID, closest[0]) > d_safe) {
      for (size_t i = 0; i < state_ID.size(); i++) {
        get_lattice_motion(ID, state_ID[i], v_x, v_y);
      }
      v_x = v_x / (float)state_ID.size();
      v_y = v_y / (float)state_ID.size();
    } else {
      get_lattice_motion(ID, state_ID[0], v_x, v_y);
    }
  }
}
