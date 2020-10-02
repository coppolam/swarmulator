#include "onlinelearning_pfsm.h"
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
#define ITER_MAX 10

/**
 * BOOST (float)
 *
 * This boosts the learning by amplyfing higher probabilities,
 * enabling faster convergence to a solution.
 *
 * Use 1.0 for no boost (will be slower learning)
 */
#define BOOST 2.

/*****************************************************************************/

onlinelearning_pfsm::onlinelearning_pfsm():
  Controller(),
  t((int)sqrt(param->pr_states())), // 4 sensors around the robot
  p(param->pr_states(), param->pr_actions())
{
  vmean = 0.5; // Adjustment velocity
  timelim = 5. * param->simulation_updatefreq();
  selected_action = 3; // Initial action
  st = 100; // Initialize to dummy state
  states = param->pr_states();
  actions = param->pr_actions();

  // Initialize policy
  policy = ones(states, actions) / (float)actions;

  // Initialize timer at random time within decision interval
  moving_timer = rg.uniform_int(0, timelim);

  moving = false; // False, the robot has not yet selected an action
  done = true; // True, the optimization routine is "done" (idle)

  // Initialize individual
  p.init(false);
}

void onlinelearning_pfsm::optimization_routine_ref(
  pagerank_estimator p, // Data
  mat &policy, // Policy reference
  bool &f) // Flag indicating completion
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // Prepare policy matrix into a vector
  vec temp = vectorise(policy); // Flatten policy to vector

  // If we have some data in our model, run the optimization
  if (any(any(p.H))) {

    // Nealder-Mead optimization routine
    optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings);

    // Apply boost
    temp.for_each([](mat::elem_type & x) { x = pow(x, BOOST); });

    // Reshape and normalize
    policy = normalise(reshape(temp, p.n_states, p.n_actions), 1, 1);
  }

  // Set optimization done flag to true
  f = true;
}

mat onlinelearning_pfsm::optimization_routine(pagerank_estimator p, mat policy)
{
  // Set up optimization settings
  optim::algo_settings_t settings;
  optimization_settings(settings);
  settings.iter_max = ITER_MAX;

  // Prepare policy matrix into a vector
  vec temp = vectorise(policy); // Flatten policy to vector

  // If we have some data in our model, run the optimization
  if (any(any(p.H))) {

    // Nealder-Mead optimization routine
    optim::nm(temp, onlinelearning_pfsm::fitness, &p, settings);

    // Apply boost
    temp.for_each([](mat::elem_type & x) { x = pow(x, BOOST); });

    // Reshape and normalize
    policy = normalise(reshape(temp, p.n_states, p.n_actions), 1, 1);
  }

  return policy;
}

double onlinelearning_pfsm::fitness(const vec &inputs,
                                    vec *grad_out,
                                    void *data)
{
  // Load up pagerank estimator as pointer
  pagerank_estimator *p = reinterpret_cast<pagerank_estimator *>(data);

  // Determine pagerank
  mat policy(inputs); // Get policy vector back into matrix form
  policy.for_each([](mat::elem_type & x) { x = pow(x, BOOST); }); // Boost
  policy = normalise(reshape(policy, p->n_states, p->n_actions), 1, 1); // Normalize rows

  // Update model based on policy
  mat H = zeros(p->n_states, p->n_states);
  for (uint m = 0; m < p->n_actions; m++) {
    // If we have a model for the effects of the action, we update
    // Else we skip, just to save some time;
    if (any(any(p->A[m]))) {
      mat temp = normalise(p->A[m], 1, 1);
      temp.each_col() %= policy.col(m);
      H += temp;
    }
  }

  // Get pagerank (assume alpha=1.0, no env. model)
  mat pr = pagerank(normalise(H, 1, 1));

  // Calculate fitness

  // Desired states
  mat des = ones(p->n_states, 1);
  des[0] = 0;

  // Pagerank fitness (negative to maximize instead of minimize)
  return -(dot(pr.t(), des) / mean(mean(des))) / mean(mean(pr));
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

  v_x = 0.0;
  v_y = 0.0;
  get_lattice_motion_all(ID, v_x, v_y);

  std::vector<bool> sensor;
  std::vector<int> temp;
  t.assess_situation(ID, sensor, temp);
  if (st != bool2int(sensor) || moving_timer == 1) { // on state change
    st = bool2int(sensor);

    // Onboard estimator
    uint a = 0;
    if (moving) {
      a = selected_action + 1;
    }
#if SHARED_MODEL
    pr.update(ID, st, a); // Use the individual model
#else
    p.update(0, st, a); // Use the shared model
#endif
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

    // Update the policy to most recent value.
    cout << ID << endl; policy.print();

#ifdef LOG
    // Open the logfile for writing
    vec v = vectorise(policy);
    for (uint16_t i = 0; i < states * actions; i++) {
      logfile << v[i] << " ";
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
    vec v = vectorise(policy);
    for (uint16_t i = 0; i < states * action; i++) {
      logfile << v[i] << " ";
    }
    logfile << endl;
#endif
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