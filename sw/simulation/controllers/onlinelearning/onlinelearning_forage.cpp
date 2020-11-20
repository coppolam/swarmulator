#include "onlinelearning_forage.h"
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
 * DETACH (boolean parameter)
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
#define DETACH false

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
#define SHARED_MODEL true

/**
 * ITER_MAX (integer)
 *
 * The maximum number of nealder-mead iteration steps for a given time steps.
 *
 * Since we pick up where we left off at the next time-step,
 * a low value like 10 will work quite well
 */
#define ITER_MAX 10

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


onlinelearning_forage::onlinelearning_forage():
  Controller(),
  p(param->pr_states(), param->pr_actions())
{
  // Control values
  timelim = 10.0 * param->simulation_updatefreq();
  vmean = 0.5;

  // Initial values
  timer = rg.uniform_int(0, timelim);
  choose = false;
  holds_food = false;
  v_x_ref = vmean;
  v_y_ref = wrapToPi_f(rg.gaussian_float(0., 0.2));
  state = environment.nest;
  keepbounded<float>(state, 0., 29.);
  st = int(state);

  states = param->pr_states();
  actions = param->pr_actions();
  policy = ones(states) / 2.;
  // Initialize individual
  p.init(false);
  done = true;
}


void onlinelearning_forage::optimization_routine_ref(
  pagerank_estimator p, // Data
  vec &policy, // Policy reference
  bool &f) // Flag indicating completion
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // If we have some data in our model, run the optimization
  if (any(any(p.H)) || any(any(p.E))) {

    // Nealder-Mead optimization routine
    optim::nm(policy, onlinelearning_forage::fitness, &p, settings);

    // Apply boost
    policy.for_each([](vec::elem_type & x) { x = pow(x, BOOST); });
  }

  // Done because the optimizer otherwise brings instability due to bounds
  policy = clamp(policy, 0.0, 1.0);

  // Set optimization done flag to true
  f = true;
}

vec onlinelearning_forage::optimization_routine(pagerank_estimator p, vec policy)
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // If we have some data in our model, run the optimization
  if (any(any(p.H))) {

    // Nealder-Mead optimization routine
    optim::nm(policy, onlinelearning_forage::fitness, &p, settings);

    // Apply boost
    policy.for_each([](vec::elem_type & x) { x = pow(x, BOOST); });
  }

  return policy;
}

double onlinelearning_forage::fitness(const vec &inputs, vec *grad_out, void *data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(data);

  // Determine pagerank
  mat m(inputs);
  mat H = normalise(p->H, 1, 1);
  H.each_col() %= m;
  mat E = normalise(p->E, 1, 1);
  mat pr = pagerank(normalise(H + E, 1, 1));
  // Calculate fitness

  // Desired states
  mat des = ones(p->n_states, 1);
  des.submat(0, 0, 14, 0) = mat(15, 1);
  // Pagerank fitness (negative to maximize instead of minimize)
  return -(dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
}

void onlinelearning_forage::get_velocity_command(const uint16_t ID, float &v_x, float &psi_rate)
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
  psi_rate = 0;
  float temp, br;
  get_lattice_motion_all(ID, v_x, temp); // Repulsion from neighbors

  // Make the choice. To explore or not to explore?
  if (!choose) {
    // Go to beacon
    o.beacon(ID, br, v_y_ref); // get distance + angle to beacon
    v_x_ref = br;
    v_y_ref = 0.5 * wrapToPi_f(v_y_ref); // gain on control
    if (br < 2 * rangesensor) { // Drop the food if you are in the vicinity of the nest
      choose = true;
      float state_temp = (environment.nest - state) * 3 + 15;
      keepbounded<float>(state_temp, 0., 29.);
      st = int(state_temp);

      // Onboard estimator
      int a;
      if (moving) {a = 1;} else {a = 0;}
#if SHARED_MODEL
      pr.update(ID, st, a); // Use the individual model
#else
      p.update(0, st, a); // Use the shared model
#endif
      state = environment.nest;

      // Decide whether to explore or not based on policy
      if (rg.bernoulli(policy[st])) {
        explore = true;
      } else {
        explore = false;
      }
    }
  }

  // Behavior
  if (explore) {
    if (timer == 1) { // Go explore, change direction every new timer instance
      v_x_ref = vmean;
      v_y_ref = wrapToPi_f(rg.gaussian_float(0., 0.2));
      environment.eat_food(0.1);
    }
    uint16_t ID_food; // for sim purposes, used to delete the correct food item once grabbed
    if (holds_food) {
      o.beacon(ID, br, v_y_ref); // get distance + angle to beacon
      v_x_ref = br;
      v_y_ref = 0.5 * wrapToPi_f(v_y_ref); // gain on control
      if (br < 2 * rangesensor) { // Drop the food if you are in the vicinity of the nest
        environment.drop_food();
        holds_food = false;
        choose = false;
        timer = 1; // reset timer
      }
    } else if (o.sense_food(ID, ID_food)) {
      environment.grab_food(ID_food); // Grab the food item ID_food
      holds_food = true;
    }
  } else { // don't explore
    o.beacon(ID, br, v_y_ref); // get distance + angle to beacon
    v_x_ref = br;
    v_y_ref = 0.5 * wrapToPi_f(v_y_ref); // gain on control
    if (timer == 1) {choose = false;}
    if (br < 2 * rangesensor) {
      environment.eat_food(0.001);
    } // eat some food to keep alive
  }
  increase_counter_to_value(timer, timelim, 1);

  // Optimize policy (non-blocking version)
  if (timer == 1 && done) {
    done = false;
    // Set up a thread to run the optimization
#if SHARED_MODEL
    std::thread thr(this->optimization_routine_ref, pr,
                    std::ref(policy), std::ref(done));
#else
// cout << "opt" << endl;
    std::thread thr(this->optimization_routine_ref, p,
                    std::ref(policy),
                    std::ref(done));
#endif
    policy.print();

#if DETACH
    thr.detach();
#else
    thr.join();
#endif

  }

#ifdef LOG
  // Open the logfile for writing
  for (uint16_t i = 0; i < states * actions; i++) {
    logfile << policy[i] << " ";
  }
  logfile << endl;
#endif

  // Final output
  v_x += v_x_ref;
  psi_rate += v_y_ref;
  wall_avoidance_turn(ID, v_x, psi_rate);
}

void onlinelearning_forage::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}