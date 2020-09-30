#include "onlinelearning_pfsm.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"
#include <future>

// Use the Eigen library for optimization
#define OPTIM_ENABLE_ARMA_WRAPPERS
#include "optim.hpp"
#include "tools.h"

using namespace std;
using namespace arma;

void optimization_routine_ref(pagerank_estimator p, mat &policy, bool &f)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = arma::zeros(16 * 8, 1);
  settings.upper_bounds = arma::ones(16 * 8, 1);
  arma::vec temp = arma::vectorise(policy); // Flatten policy
  optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings); // Optimization
  policy = arma::normalise(reshape(temp, 16, 8), 1, 1); // Reshape
  f = true;
}

mat optimization_routine(pagerank_estimator p, mat policy)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = arma::zeros(16 * 8, 1);
  settings.upper_bounds = arma::ones(16 * 8, 1);
  arma::vec temp = arma::vectorise(policy); // Flatten policy
  if (any(any(p.H))) {
    optim::lbfgs(temp, onlinelearning_pfsm::fitness, &p, settings); // Optimization
  }
  return arma::normalise(reshape(temp, 16, 8), 1, 1); // Reshape and return policy

}

onlinelearning_pfsm::onlinelearning_pfsm():
  Controller(),
  t(4), // 4 sensors around the robot
  p(16, 8) // Policy 16 states (2^4), 8 actions
{
  vmean = 0.5; // Adjustment velocity
  timelim = 5. * param->simulation_updatefreq();
  selected_action = 3; // Initial action
  st = 100; // Init

  // Get policy
  policy.load(param->policy());
  // policy.print();
  moving_timer = rg.uniform_int(0, timelim);
  moving = false;
  done = true;
  // Initialize individual
  p.init(false);
}


double onlinelearning_pfsm::fitness(const vec &inputs, vec *grad_out, void *opt_data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(opt_data);

  // Determine pagerank
  arma::mat policy(inputs); // Get into matrix form
  policy.reshape(16, 8); // Reshape to correct dimensions
  policy = arma::normalise(policy, 1, 1); // Normalize rows

  arma::mat H = arma::zeros(16, 16);
  for (uint m = 0; m < 8; m++) {
    if (any(any(p->A[m]))) {
      arma::mat temp = arma::normalise(p->A[m], 1, 1);
      // cout << " " << endl; temp.print();
      // cout << " " << endl; policy.print();
      temp.each_col() %= policy.col(m);
      // cout << " " << endl; temp.print();
      H += temp;
    }
  }

  // Get pagerank (assume alpha=1.0 for now)
  arma::mat pr = pagerank(arma::normalise(H, 1, 1));

  // Calculate fitness
  arma::mat des = arma::ones(16, 1);
  des[0] = 0;
  return (arma::dot(pr.t(), des) / arma::mean(arma::mean(des))) / arma::mean(arma::mean(pr));
}


void onlinelearning_pfsm::action_motion(const int &selected_action, float r, float t, float &v_x, float &v_y)
{
  std::vector<float> ang = {-1.0, -0.7, -0.3, -0.1, 0.1, 0.3, 0.7, 1.0};
  v_x = vmean;
  v_y = ang[selected_action];
}

void onlinelearning_pfsm::state_action_lookup(const int ID, uint state_index)
{
  // Get policy distribution for the current action
  rowvec p = policy.row(state_index);

  // to float
  std::vector<float> pfloat(p.begin(), p.end());

  // Get vector from distribution
  selected_action = rg.discrete_int(pfloat);
}

void onlinelearning_pfsm::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{

  v_x = 0.0;
  v_y = 0.0;
  get_lattice_motion_all(ID, v_x, v_y);

  std::vector<bool> sensor; // 4 sensors
  std::vector<int> temp;
  t.assess_situation(ID, sensor, temp);
  if (st != bool2int(sensor) || moving_timer == 1) { // on state change
    st = bool2int(sensor);

    // Onboard estimator
    int a;
    if (moving) {a = 1;} else {a = 0;}
    p.update(0, st, a); // Update model

    state_action_lookup(ID, st);
    float r, t;
    cart2polar(s[ID]->state[2], s[ID]->state[3], r, t);
    action_motion(selected_action, r, t, vx_ref, vy_ref);
  }

  increase_counter_to_value(moving_timer, timelim, 1);

  // Optimize policy (non-blocking version)
#ifdef NON_BLOCKING
  if (moving_timer == 1 && done) {
    done = false;

    // Set up a detached thread to run in the background
    std::thread thr(optimization_routine_ref, p, std::ref(policy), std::ref(done));

    // Detach it
    thr.detach();

    // Update the policy to most recent value.
    // policy.print();
  }
#else
  // Optimize policy (blocking version)
  if (moving_timer == 1) {
    std::future<arma::mat> w = std::async(optimization_routine, p, policy);
    policy = w.get(); // blocking call
    if (ID == 0) {
      cout << ID << endl;
      policy.print();
    }
  }
#endif

  v_x += vx_ref;
  v_y += vy_ref;
  wall_avoidance_turn(ID, v_x, v_y);

  moving = true;
}

void onlinelearning_pfsm::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}