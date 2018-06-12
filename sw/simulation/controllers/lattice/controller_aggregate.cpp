#include "controller_aggregate.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

#include <algorithm> // std::sort

Controller_Aggregate::Controller_Aggregate() : Controller_Lattice_Basic()
{
  string s = "./conf/state_action_matrices/state_action_matrix_free.txt";
  t.set_state_action_matrix(s);
  timelim = 1.8 * param->simulation_updatefreq();
  moving_timer = rand() % (int)timelim;
  beta_des.push_back(0.0);
  beta_des.push_back(M_PI / 4.0);
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(3.0 * M_PI / 4.0);
}

void Controller_Aggregate::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<bool> state;
  vector<int>  state_ID; // The ID is just used for simulation purposes
  t.assess_situation(ID, state, state_ID);
  int state_index = bool2int(state);
  int n_neighbors = state_ID.size(); // number of neighbors
  int n_cliques = 0;
  bool onclique;

  // This only makes sense if you have between 2 and 6 neighbors, else #cliques = 1;
  if (n_neighbors > 1 && n_neighbors < 7) {
    onclique = false;
    for (size_t i = 0; i < state.size(); i++) {
      if (state[i] && !onclique) {
        onclique = true;
        n_cliques++;
      } else if (!state[i]) {
        onclique = false;
      }
    }
    if (state[0] && state[state.size() - 1] && n_cliques > 1) {
      n_cliques--;
    }
  } else {
    n_cliques = 1;
  }

  // TODO: Move these elsewhere
  int twait_1 = timelim * 2 ; //(n_neighbors) / (n_cliques) ;
  int twait_2 = twait_1 * 2 ; //(n_neighbors) / (n_cliques);

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // Can I move or are my neighbors moving?
  bool canImove = true;
  for (uint8_t i = 0; i < state_ID.size(); i++) {
    if (o->see_if_moving(state_ID[i])) {
      // Somebody nearby is already moving
      canImove = false;
      selected_action = -2; // Reset actions
      moving_timer = twait_1; // Reset moving timer
    }
  }

  // Action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t.state_action_matrix.find(state_index);

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  if (!o->see_if_moving(ID)) {
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
      selected_action = -2; // State not found... no action to take
    }
  }

  // Controller Logic
  moving = false;
  if (canImove) {
    if (selected_action > -1 && moving_timer < timelim && o->request_distance(ID, closest[0]) < 1.6) {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    } else {
      get_lattice_motion_all(ID, state_ID, closest, v_x, v_y);
    }
    increase_counter(moving_timer, twait_2);
  }
}
