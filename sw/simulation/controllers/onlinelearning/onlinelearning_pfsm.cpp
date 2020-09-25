#include "onlinelearning_pfsm.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"

// Use the Eigen library for optimization
#define OPTIM_ENABLE_ARMA_WRAPPERS
#include "optim.hpp"
#include "tools.h"

using namespace std;
using namespace arma;

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
  policy = arma::normalise(policy); // Normalize rows

  arma::mat H = arma::zeros(16, 16);
  for (uint m = 0; m < 8; m++) {
    arma::mat temp = arma::normalise(p->A[m], 1, 1);
    temp.each_row() %= policy.col(m).t();
    H += temp;
  }
  arma::mat pr = pagerank(
                   arma::normalise(H + p->E, 1, 1)
                 ); // assume alpha=0.5

  // Calculate fitness
  arma::mat des = ones(16, 1);
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

  // Optimize policy
  if (moving_timer == 1) {
    optim::algo_settings_t settings;
    settings.vals_bound = true;
    settings.upper_bounds = arma::ones(16 * 8, 1);
    settings.lower_bounds = arma::zeros(16 * 8, 1);
    arma::vec temp = vectorise(policy); // Flatten policy
    optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings); // Optimization
    policy = normalise(reshape(temp, 16, 8)); // Reshape
    // cout << "mat" << endl;
    // policy.print(); // debug
  }

  v_x += vx_ref;
  v_y += vy_ref;
  wall_avoidance_turn(ID, v_x, v_y);

  // cout << v_x << " " << v_y << endl;

  moving = true;
}

void onlinelearning_pfsm::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}