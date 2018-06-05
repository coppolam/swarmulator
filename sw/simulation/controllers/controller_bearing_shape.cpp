#include "controller_bearing_shape.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "auxiliary.h"
#include "kill_functions.h"

#include <algorithm> // std::find

Controller_Bearing_Shape::Controller_Bearing_Shape() : Controller(),
  moving(nagents, 0),
  moving_timer(nagents, 0),
  selected_action(nagents, -1),
  happy(nagents, 0)
{

}

float Controller_Bearing_Shape::f_attraction(float u, float b_eq)
{
  float w;
  if (!(abs(b_eq - M_PI / 4.0) < 0.1 || abs(b_eq - (3 * M_PI / 4.0)) < 0.1)) {
    w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
  } else {
    w = log((sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)) / _kr - 1) / exp(-_ka * sqrt(pow(_ddes, 2.0) + pow(_ddes, 2.0)))) / _ka;
  }

  return 1 / (1 + exp(-_ka * (u - w)));
}

float Controller_Bearing_Shape::get_attraction_velocity(float u, float b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u);
}

void Controller_Bearing_Shape::attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}

void Controller_Bearing_Shape::latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}

void Controller_Bearing_Shape::actionmotion(const int selected_action, float &v_x, float &v_y)
{
  float actionspace_y[8] = {0, sqrt(1), 1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1)};
  float actionspace_x[8] = {1, sqrt(1), 0, -sqrt(1), -1, -sqrt(1), 0, sqrt(1)};
  v_x = _v_adj * actionspace_x[selected_action];
  v_y = _v_adj * actionspace_y[selected_action];
}

void Controller_Bearing_Shape::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  float timelim = 1.8 * param->simulation_updatefreq();
  float twait_1 = timelim * 2;
  float twait_2 = twait_1 * 2;
  float v_r, b_eq, v_b;

  vector<float> beta_des;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);

  // Initialize moving_timer with random variable
  if (moving_timer[ID] == 0) {
    moving_timer[ID] = rand() % (int)timelim;
  }

  vector<bool> state(8, 0);
  vector<int>  state_ID;
  // The ID is just used for simulation purposes
  t->assess_situation(ID, state, state_ID);
  int state_index = bool2int(state);

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // Can I move or are my neighbors moving?
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (moving[state_ID[i]]) {
      canImove = false;
      selected_action[ID] = -2; // Reset actions
      moving_timer[ID] = twait_1; // Reset moving timer
    }
  }

  vector<uint> sdes = {3, 28, 31, 96, 124, 163, 190, 226, 227};           // todo: make this not a hack
  vector<float> priority = {5, 3, 4, 1, 2, 4, 3, 2, 3};          // todo: make this not a hack
  // vector<uint> sdes = { 3, 28, 96, 162 };
  // vector<uint> priority = {3, 2, 1, 2}; // todo: make this not a hack

  if (std::find(sdes.begin(), sdes.end(), state_index) != sdes.end()) {
    happy[ID] = true;
  } else {
    happy[ID] = false;
  }

  int s = 0;
  for (uint8_t i = 0; i < nagents; i++) {
    s += happy[i];
  }
  if (s == nagents) {
    killer k;
    // k.kill_switch();
  }

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t->state_action_matrix.find(state_index);
  if (!moving[ID] && state_action_row != t->state_action_matrix.end()) {
    selected_action[ID] = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
  } else if (!moving[ID]) {
    selected_action[ID] = -2;
  }

  // Controller
  moving[ID] = false;
  float d_safe = 0.9;

  if (canImove) {
    if (selected_action[ID] > -1 && moving_timer[ID] < timelim && o->request_distance(ID, closest[0]) < 1.2) {
      actionmotion(selected_action[ID], v_x, v_y);
      moving[ID] = true;
    } else if (o->request_distance(ID, closest[0]) > d_safe) {
      uint count = 1;
      for (size_t i = 0; i < state_ID.size(); i++) {
        v_b = wrapToPi_f(o->request_bearing(ID, state_ID[i]));
        b_eq = t->get_preferred_bearing(beta_des, v_b);
        v_r = get_attraction_velocity(o->request_distance(ID, state_ID[i]), b_eq);
        latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
        count++;
      }
      v_x = v_x / (float)count;
      v_y = v_y / (float)count;
    } else {
      v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
      b_eq = t->get_preferred_bearing(beta_des, v_b);
      v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);
      latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
    }

    if (moving_timer[ID] > twait_2) {
      moving_timer[ID] = 1;
    } else {
      moving_timer[ID]++;
    }
  }

  keepbounded(v_x, -1, 1);
  keepbounded(v_y, -1, 1);
}