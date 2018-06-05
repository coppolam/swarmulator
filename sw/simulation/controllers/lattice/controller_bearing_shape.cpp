#include "controller_bearing_shape.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "auxiliary.h"
#include "kill_functions.h"

#include <algorithm> // std::find

Controller_Bearing_Shape::Controller_Bearing_Shape() : Controller_Lattice_Basic()
{
  string s = "./conf/state_action_matrices/state_action_matrix_triangle4.txt";
  t.set_state_action_matrix(s);
  moving_timer = 0;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);
}

void Controller_Bearing_Shape::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  float timelim = 1.8 * param->simulation_updatefreq();
  float twait_1 = timelim * 2;
  float twait_2 = twait_1 * 2;
  float v_r, b_eq, v_b;

  // Initialize moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rand() % (int)timelim;
  }

  vector<bool> state(8, 0);
  vector<int>  state_ID;
  // The ID is just used for simulation purposes
  t.assess_situation(ID, state, state_ID);
  int state_index = bool2int(state);

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // Can I move or are my neighbors moving?
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (o->see_if_moving(state_ID[i]))
    {
      canImove = false;
      selected_action = -2; // Reset actions
      moving_timer = twait_1; // Reset moving timer
    }
  }

  // vector<uint> sdes = {3, 28, 31, 96, 124, 163, 190, 226, 227};           // todo: make this not a hack
  // vector<uint> sdes = { 3, 28, 96, 162 };
  
  // if (std::find(sdes.begin(), sdes.end(), state_index) != sdes.end()) {
  //   happy[ID] = true;
  // } else {
  //   happy[ID] = false;
  // }

  // int s = 0;
  // for (uint8_t i = 0; i < nagents; i++) {
  //   s += happy[i];
  // }
  // if (s == nagents) {
  //   killer k;
    // k.kill_switch();
  // }

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t.state_action_matrix.find(state_index);
  if (!o->see_if_moving(ID) && state_action_row != t.state_action_matrix.end())
  {
    selected_action = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
  }
  else if (!o->see_if_moving(ID))
  {
    selected_action = -2;
  }

  // Controller
  moving = false;
  float d_safe = 0.9;

  if (canImove) {
    if (selected_action > -1 && moving_timer < timelim && o->request_distance(ID, closest[0]) < 1.2) {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    } else if (o->request_distance(ID, closest[0]) > d_safe) {
      uint count = 1;
      for (size_t i = 0; i < state_ID.size(); i++) {
        v_b = wrapToPi_f(o->request_bearing(ID, state_ID[i]));
        b_eq = t.get_preferred_bearing(beta_des, v_b);
        v_r = get_attraction_velocity(o->request_distance(ID, state_ID[i]), b_eq);
        latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
        count++;
      }
      v_x = v_x / (float)count;
      v_y = v_y / (float)count;
    } else {
      v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
      b_eq = t.get_preferred_bearing(beta_des, v_b);
      v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);
      latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
    }

    if (moving_timer > twait_2) {
      moving_timer = 1;
    } else {
      moving_timer++;
    }
  }

  keepbounded(v_x, -1, 1);
  keepbounded(v_y, -1, 1);
}