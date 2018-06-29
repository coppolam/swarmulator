#include "controller_aggregate.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

#include <algorithm> // std::sort

Controller_Aggregate::Controller_Aggregate() : Controller_Lattice_Basic()
{
  string s = "./conf/state_action_matrices/state_action_matrix_free_test1.txt";
  t.set_state_action_matrix(s);
  timelim = 1.8 * param->simulation_updatefreq();
  twait = timelim * 3.0;
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
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest
  bool canImove = check_motion(state_ID);

  // Action selection
  std::map<int, vector<int>>::iterator state_action_row = t.state_action_matrix.find(bool2int(state));
  bool action_available = true;
  if (!o->see_if_moving(ID) && state_action_row != t.state_action_matrix.end()) {
    selected_action = *select_randomly(state_action_row->second.begin(), state_action_row->second.end());
  } else if (!o->see_if_moving(ID)){
    action_available = false;
  }

  if (!canImove){
    action_available = false;
    moving_timer = timelim;
  }

  // Controller Logic
  moving = false;
  if (canImove) {
    if (action_available && moving_timer < timelim && o->request_distance(ID, closest[0]) < rangesensor) {
      actionmotion(selected_action, v_x, v_y);
      moving = true;
    } else {
      get_lattice_motion_all(ID, state_ID, closest, v_x, v_y);
    }
    increase_counter(moving_timer, twait);
  }
}
