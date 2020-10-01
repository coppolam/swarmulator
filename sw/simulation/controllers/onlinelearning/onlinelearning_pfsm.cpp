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
#define NON_BLOCKING TRUE
using namespace std;
using namespace arma;

void optimization_routine_ref(pagerank_estimator p, mat &policy, bool &f)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = zeros(16 * 8, 1);
  settings.upper_bounds = ones(16 * 8, 1);
  vec temp = vectorise(policy); // Flatten policy
  if (any(any(p.H))) {
    optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings); // Optimization
  }
  policy = normalise(reshape(temp, 16, 8), 1, 1); // Reshape
  f = true;
}

mat optimization_routine(pagerank_estimator p, mat policy)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = zeros(16 * 8, 1);
  settings.upper_bounds = ones(16 * 8, 1);
  // settings.gd_settings.method = 1;
  // settings.gd_settings.par_step_size = 0.001;
  vec temp = vectorise(policy); // Flatten policy
  if (any(any(p.H))) { // Don't bother otherwise
    optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings); // Optimization
  }
  return normalise(reshape(temp, 16, 8), 1, 1); // Reshape and return policy
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
  policy = ones(16, 8) / 8;
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
  mat policy(inputs); // Get into matrix form
  policy.reshape(16, 8); // Reshape to correct dimensions
  policy = normalise(policy, 1, 1); // Normalize rows
  // cout << "p->H" << endl; normalise(p->H,1,1).print();
  // cout << "policy" << endl; policy.print();
  mat H = zeros(16, 16);
  for (uint m = 0; m < 8; m++) {
    if (any(any(p->A[m]))) {
      // cout << "p->A" <<m << endl; p->A[m].print();
      mat temp = normalise(p->A[m], 1, 1);
      temp.each_col() %= policy.col(m);
      // cout << "temp" << endl; temp.print();
      H += temp;
      // cout << "H" << endl; normalise(H,1,1).print();
    }
  }

  // Get pagerank (assume alpha=1.0 for now)
  mat pr = pagerank(normalise(H, 1, 1));

  // Calculate fitness
  mat des = ones(16, 1);
  des[0] = 0;
  // pr.print();
  double f = (dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
  // cout << f << endl;
  return -f;
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
    uint a = 0;
    if (moving) {a = selected_action + 1;}
    p.update(0, st, a);
    // pr.update(ID, st, a);

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
    // if (ID == 0) {
    std::future<mat> w = std::async(optimization_routine, p, policy);
    policy = w.get(); // blocking call
    // if (ID == 0) {
    // }
  }
#endif
  if (moving_timer == 1) {
    cout << ID << endl;
    policy.print();
  }
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