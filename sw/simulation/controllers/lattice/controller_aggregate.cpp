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
  twait = timelim*5.0;
  moving_timer = rand() % twait;
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

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest
  bool canImove = check_motion(state_ID);
  if (!canImove)
  {
    selected_action = -2;
    moving_timer = timelim;  
  }

  // Action
  std::map<int, vector<int>>::iterator state_action_row;
  state_action_row = t.state_action_matrix.find(state_index);

  // Try to find an action that suits the state, if available (otherwise you are in Sdes or Sblocked)
  // If you are already busy with an action, then don't change the action
  if (!o->see_if_moving(ID)) {
    if (state_action_row != t.state_action_matrix.end()) {
      if (motion_dir == 0 || state_action_row->second.size() < 2) {
        selected_action = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
      }
    } else {
      selected_action = -2; // State not found... no action to take
    }
  }
  
  // Controller Logic
  moving = false;
  if (canImove) {
    if (selected_action > -1 && moving_timer < timelim && o->request_distance(ID, closest[0]) < rangesensor) {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    } else {
      get_lattice_motion_all(ID, state_ID, closest, v_x, v_y);
    }
    increase_counter(moving_timer, twait);
  }
}
