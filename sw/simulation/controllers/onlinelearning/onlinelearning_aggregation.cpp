#include "onlinelearning_aggregation.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"
#include <future>

// Enable the Armadillo library for matrix operations
#define OPTIM_ENABLE_ARMA_WRAPPERS
// Load the optim package header
#include "optim.hpp"

// Load optimization specific tools (pagerank function)
#include "tools.h"
// Load up namespaces
using namespace std;
using namespace arma;

// Define settings of the optimization of
// 4 key simulation/optimization parameters

/* If this is set, then the optimization runs on a separate thread independently of the simulation, this means that the simulation will not wait for the optimization to finish within this time step, but rather continue and update the reference to the value at a later time
    If this is false, then the optimization will run within the time step
    */
#define NON_BLOCKING true

/* If this is true, then the agents will share a single model to optimize their controllers. If false, then each agent maintains and optimizes based on an individual model
*/
#define SHARED_MODEL false

/* The maximum number of nealder-mead iteration steps for a given time steps.
  Since we pick up where we left off at the next time-step, this works quite well
  */
#define ITER_MAX 10

/*This boosts the learning by amplyfing higher probabilities, enabling faster convergence to a solution.
Use 1 for no boost, at the cost of slower learning
*/
#define BOOST 2.


onlinelearning_aggregation::onlinelearning_aggregation(): Controller(), p(8, 1)
{
  // Initial values
  moving = false;
  moving_timer = rg.uniform_int(0, timelim);
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);

  // Control values
  timelim = 2.0 * param->simulation_updatefreq();
  vmean = 0.5;

  // Policy
  motion_p.assign(8, 0.5);

  vec temp(motion_p);
  pol = temp;

  // Initialize individual
  p.init(false);
  done = true;
}


void onlinelearning_aggregation::optimization_routine_ref(pagerank_estimator p, vec &policy, bool &f)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = zeros(8, 1);
  settings.upper_bounds = ones(8, 1);
  // If we have already recorded some data into our model, then run the optimization routine
  if (any(any(p.H))) {
    optim::nm(policy, onlinelearning_aggregation::fitness, &p, settings);
  }
  f = true;
}

vec onlinelearning_aggregation::optimization_routine(pagerank_estimator p, vec policy)
{
  optim::algo_settings_t settings;
  settings.vals_bound = true;
  settings.lower_bounds = zeros(8, 1);
  settings.upper_bounds = ones(8, 1);
  // settings.gd_settings.method = 1;
  // settings.gd_settings.par_step_size = 0.001;
  // vec temp = vectorise(policy); // Flatten policy
  if (any(any(p.H))) { // Don't bother otherwise
    optim::nm(policy, onlinelearning_aggregation::fitness, &p, settings); // Optimization
  }
  return policy;
}

double onlinelearning_aggregation::fitness(const vec &inputs, vec *grad_out, void *opt_data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(opt_data);

  // Determine pagerank
  mat m(inputs);
  mat G = normalise(p->H, 1, 1);
  G.each_col() %= m;
  mat pr = pagerank(normalise(G + p->E, 1, 1)); // assume alpha=0.5

  // Calculate fitness
  mat des = ones(8, 1);
  des[0] = 0;
  double f = (dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
  // cout << f << endl;
  return -f;
}

void onlinelearning_aggregation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<double> a = {1, 2, 3, 4};
  mat m(&a.front(), 2, 2);

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = std::min(r.size(), motion_p.size());

    // Onboard estimator
    int a;
    if (moving) {a = 1;} else {a = 0;}
    p.update(0, st, a); // Update model

    if (rg.bernoulli(1.0 - pol[st])) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      float ang = rg.uniform_float(0.0, 2 * M_PI);
      if (moving) {
        float ext = rg.gaussian_float(0.0, 0.5);
        float temp;
        cart2polar(v_x_ref, v_y_ref, temp, ang);
        ang += ext;
      }
      wrapTo2Pi(ang);
      polar2cart(vmean, ang, v_x_ref, v_y_ref);
      moving = true;
    }
  }
  increase_counter_to_value(moving_timer, timelim, 1);
  wall_avoidance_bounce(ID, v_x_ref, v_y_ref);

  // Optimize policy (non-blocking version)
#ifdef NON_BLOCKING
  if (moving_timer == 1 && done) {
    done = false;

    // Set up a detached thread to run in the background
    std::thread thr(this->optimization_routine_ref, p, std::ref(pol), std::ref(done));

    // Detach it
    thr.detach();

    cout << ID << endl; pol.t().print();
  }
#else
  // Optimize policy (blocking version)
  if (moving_timer == 1) {
    // if (ID == 0) {
    std::future<mat> w = std::async(optimization_routine, p, pol);
    pol = w.get(); // blocking call

  }
#endif
  // if (ID == 0) {
  // pol.t().print();
  // }
  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}

void onlinelearning_aggregation::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}