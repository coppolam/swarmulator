#include "controller_keep_aggregate.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

#include <algorithm> // std::sort

Controller_Keep_Aggregate::Controller_Keep_Aggregate() : Controller()
{
  string s = "./conf/state_action_matrices/state_action_matrix_triangle4.txt";
  t.set_state_action_matrix(s);
  moving_timer = 0;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);
}

float Controller_Keep_Aggregate::f_attraction(float u, float b_eq)
{
  float w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}

float Controller_Keep_Aggregate::get_attraction_velocity(float u, float b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u);
}

void Controller_Keep_Aggregate::attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x = v_r * cos(v_b);
  v_y = v_r * sin(v_b);
}

void Controller_Keep_Aggregate::latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);
  // Additional force for for reciprocal alignment
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}

void Controller_Keep_Aggregate::actionmotion(const int selected_action, float &v_x, float &v_y)
{
  int actionspace_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  int actionspace_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  v_x = _v_adj * (float)actionspace_x[selected_action];
  v_y = _v_adj * (float)actionspace_y[selected_action];
}

void Controller_Keep_Aggregate::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest
  float v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
  float b_eq = t.get_preferred_bearing(beta_des, v_b);
  float v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);

  // State
  vector<bool> state(8, 0);
  vector<int>  state_ID;
  t.assess_situation(ID, state, state_ID); // The ID is just used for simulation purposes
  int state_index = bool2int(state);

  // Can I move or are my neighbors moving?
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (o->see_if_moving(state_ID[i])) { // Somebody nearby is already moving
      canImove = false;
    }
  }

  // Action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = state_action_matrix.find(state_index);

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  if (!o->see_if_moving(ID)) {
    if (state_action_row != state_action_matrix.end()) {
      if (motion_dir == 0 || state_action_row->second.size() < 2) {
        selected_action = *select_randomly(state_action_row->second.begin(),
                                               state_action_row->second.end());
      } else {
        // Possible actions
        vector<int> possibleactions = state_action_row->second;
        for_each(possibleactions.begin(), possibleactions.end(), [](int &d) { d++; });

        // Difference to favorite action
        vector<int> difference = state_action_row->second;
        for_each(difference.begin(), difference.end(), [](int &d) { d++; });
        for_each(difference.begin(), difference.end(), [&](int &d) { d -= motion_dir; });
        for_each(difference.begin(), difference.end(), [](int &d) { d = abs(d); });
        for_each(difference.begin(), difference.end(), [](int &d)
        { if (d > 4) {d = 4 - wraptosequence(d, 0, 4);} else {d = wraptosequence(d, 0, 4);} });

        // Sort
        std::pair<int, int> AB[difference.size()];
        for (size_t i = 0; i < difference.size(); ++i) {
          AB[i].first = difference[i];
          AB[i].second = possibleactions[i];
        }
        std::sort(AB, AB + difference.size());
        for (size_t i = 0; i < difference.size(); ++i) {
          difference[i] = AB[i].first;
          possibleactions[i] = AB[i].second;
        }

        // Select
        if (difference[0] != difference[1]) {
          selected_action = possibleactions[0] - 1;
        } else {
          selected_action = *select_randomly(possibleactions.begin(),
                                                 possibleactions.begin() + 1) - 1;
        }
      }

    } else {
      selected_action = -2; // State not found... no action to take.
    }
  }

  moving = false;
  if (selected_action > -1 && canImove && moving_timer < 200) {
    // You are in not blocked and you have priority. Take an action!
    actionmotion(selected_action, v_x, v_y);
    moving = true;
    moving_timer++;
  } else if (canImove) {
    // You are static, but you still have priority! Fix your position.
    latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
    moving_timer = 0;
  } else {
    // You are static, but also too slow, so no priority! Wait about till you do.
    attractionmotion(v_r, v_b, v_x, v_y);
    moving_timer = 0;
  }

  keepbounded(v_x, -1, 1);
  keepbounded(v_y, -1, 1);
}
