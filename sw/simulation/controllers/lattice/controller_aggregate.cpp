#include "controller_aggregate.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

#include <algorithm> // std::sort

Controller_Aggregate::Controller_Aggregate() : Controller_Lattice_Basic()
{
  string s = "./conf/state_action_matrices/state_action_matrix_free.txt";
  t.set_state_action_matrix(s);
  moving_timer = 0;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);
}



void Controller_Aggregate::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{

  float timelim = 1.8 * param->simulation_updatefreq();
  float twait_1 = timelim * 2;
  float twait_2 = twait_1 * 2;

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
    if (o->see_if_moving(state_ID[i]))
    { // Somebody nearby is already moving
      canImove = false;
    }
  }

  // Action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t.state_action_matrix.find(state_index);

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  if (!o->see_if_moving(ID))
  {
    if (state_action_row != t.state_action_matrix.end()) {
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
        for_each(difference.begin(), difference.end(), [&](int &d) { d -= motion_dir; }); // https://stackoverflow.com/questions/13461538/lambda-of-a-lambda-the-function-is-not-captured/13461594
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

  // Controller
  moving = false;
  float d_safe = 0.9;

  if (canImove)
  {
    if (selected_action > -1 && moving_timer < timelim && o->request_distance(ID, closest[0]) < 1.2)
    {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    }
    else if (o->request_distance(ID, closest[0]) > d_safe)
    {
      uint count = 1;
      for (size_t i = 0; i < state_ID.size(); i++)
      {
        v_b = wrapToPi_f(o->request_bearing(ID, state_ID[i]));
        b_eq = t.get_preferred_bearing(beta_des, v_b);
        v_r = get_attraction_velocity(o->request_distance(ID, state_ID[i]), b_eq);
        latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
        count++;
      }
      v_x = v_x / (float)count;
      v_y = v_y / (float)count;
    }
    else
    {
      v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
      b_eq = t.get_preferred_bearing(beta_des, v_b);
      v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);
      latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
    }

    if (moving_timer > twait_2)
    {
      moving_timer = 1;
    }
    else
    {
      moving_timer++;
    }
  }

  keepbounded(v_x, -1, 1);
  keepbounded(v_y, -1, 1);
}
