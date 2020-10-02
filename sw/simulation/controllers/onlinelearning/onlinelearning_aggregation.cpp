#include "onlinelearning_aggregation.h"
#include <future>
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"

// Load optimization specific tools (pagerank function)
#include "tools.h"

// Load up namespaces
using namespace std;
using namespace arma;

/***********************************
 * SETTINGS
 * *********************************/

// Define settings of the optimization of
// 4 key simulation/optimization parameters

/**
 * NON_BLOCKING (boolean parameter)
 *
 *  - If true: the optimization runs on a separate thread independently of
 *             the simulation, this means that the simulation will not wait
 *             for the optimization to finish within this time step, but
 *             rather continue and update the reference to the value at
 *             a later time.
 *
 *  - If false: the optimization will run within the time step
 *
 */
#define NON_BLOCKING true

/**
 * SHARED_MODEL (boolean parameter)
 *
 * - If true: the agents will share a single model to optimize
 *            their Controllers.
 *
 * - If false: each agent maintains and optimizes based on an
 *             individual model
 *
 */
#define SHARED_MODEL false

/**
 * ITER_MAX (integer)
 *
 * The maximum number of nealder-mead iteration steps for a given time steps.
 *
 * Since we pick up where we left off at the next time-step,
 * a low value like 10 will work quite well
 */
#define ITER_MAX 20

/**
 * BOOST (float)
 *
 * This boosts the learning by amplyfing higher probabilities,
 * enabling faster convergence to a solution.
 *
 * Use 1.0 for no boost (will be slower learning)
 */
#define BOOST 1.

/*****************************************************************************/


onlinelearning_aggregation::onlinelearning_aggregation():
  Controller(),
  p(param->pr_states(), param->pr_actions())
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
  states = param->pr_states();
  actions = param->pr_actions();
  policy = ones(states) / (float)actions;

  // Initialize individual
  p.init(false);
  done = true;
}


void onlinelearning_aggregation::optimization_routine_ref(
  pagerank_estimator p, // Data
  vec &policy, // Policy reference
  bool &f) // Flag indicating completion
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // If we have some data in our model, run the optimization
  if (any(any(p.H))) {

    // Nealder-Mead optimization routine
    optim::nm(policy, onlinelearning_aggregation::fitness, &p, settings);

    // Apply boost
    policy.for_each([](vec::elem_type & x) { x = pow(x, BOOST); });
  }

  // Set optimization done flag to true
  f = true;
}

vec onlinelearning_aggregation::optimization_routine(pagerank_estimator p, vec policy)
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // If we have some data in our model, run the optimization
  if (any(any(p.H))) {

    // Nealder-Mead optimization routine
    optim::nm(policy, onlinelearning_aggregation::fitness, &p, settings);

    // Apply boost
    policy.for_each([](vec::elem_type & x) { x = pow(x, BOOST); });
  }

  return policy;
}

double onlinelearning_aggregation::fitness(const vec &inputs, vec *grad_out, void *data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(data);

  // Determine pagerank
  mat m(inputs);
  mat G = normalise(p->H, 1, 1);
  G.each_col() %= m;
  mat pr = pagerank(normalise(G + p->E, 1, 1)); // assume alpha=0.5

  // Calculate fitness

  // Desired states
  mat des = ones(p->n_states, 1);
  des[0] = 0;

  // Pagerank fitness (negative to maximize instead of minimize)
  return -(dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
}

void onlinelearning_aggregation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{

#ifdef LOG
  // Initialize policy logfile (st=100 is a dummy initial state)
  // Set up a unique name for the file and open it for writing.
  if (st == 100) {
    stringstream s;
    s << ID;
    filename = "logs/policy_" + identifier + "_" + s.str() + ".txt";
    writer.setfilename(filename); // Set the filename
    logfile.open(filename.c_str()); // Open for writing
  }
#endif

  v_x = 0;
  v_y = 0;

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = std::min<int>(r.size(), states);

    // Onboard estimator
    int a;
    if (moving) {a = 1;} else {a = 0;}
#if SHARED_MODEL
    pr.update(ID, st, a); // Use the individual model
#else
    p.update(0, st, a); // Use the shared model
#endif

    if (rg.bernoulli(1.0 - policy[st])) {
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

  // Optimize policy (non-blocking version)
#ifdef NON_BLOCKING
  if (moving_timer == 1 && done) {
    done = false;

    // Set up a thread to run the optimization
#if SHARED_MODEL
    std::thread thr(this->optimization_routine_ref, pr,
                    std::ref(policy),
                    std::ref(done));
#else
    std::thread thr(this->optimization_routine_ref, p,
                    std::ref(policy),
                    std::ref(done));
#endif

    // Detach it
    thr.detach();

    // // Update the policy to most recent value.
    cout << ID << endl; policy.t().print();

#ifdef LOG
    // Open the logfile for writing
    for (uint16_t i = 0; i < states * actions; i++) {
      logfile << policy[i] << " ";
    }
    logfile << endl;
#endif
  }

#else
  // Optimize policy (blocking version with async)
  if (moving_timer == 1) {
    std::future<mat> w = std::async(optimization_routine, p, policy);
    policy = w.get(); // Blocking call
#ifdef LOG
    // Open the logfile for writing
    for (uint16_t i = 0; i < states * actions; i++) {
      logfile << policy[i] << " ";
    }
    logfile << endl;
#endif
  }
#endif

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
  wall_avoidance_bounce(ID, v_x_ref, v_y_ref);
}

void onlinelearning_aggregation::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}