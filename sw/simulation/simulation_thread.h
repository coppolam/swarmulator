#ifndef SIMULATION_H
#define SIMULATION_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <thread>         // std::thread
#include <mutex>

#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include "kill_functions.h"
#include "agent_thread.h"
#include "settings.h"

bool simulation_running = false;

void run_simulation()
{
  if (!simulation_running) {
    terminalinfo ti;
    ti.info_msg("Simulation started.");
    simulation_running = true;
  }

  int t_wait = (int) 1000000.0 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
  this_thread::sleep_for(chrono::microseconds(t_wait));
  simulation_time = t_wait;
  simtime_seconds += param->simulation_realtimefactor() * simulation_time / 1000000.0;
}

/* Calculate the mean of a vector element */
float vector_mean(const vector<float> &v)
{
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

/* Calculate the standard deviation of a vector element */
float vector_std(const vector<float> &v)
{
  vector<double> diff(v.size());
  std::transform(
    v.begin(), v.end(), diff.begin(),
    std::bind2nd(std::minus<double>(), vector_mean(v))
  );
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return sqrt(sq_sum / v.size());
}

void simulation_start()
{
  simulation_time = 0.0; // Initialize simulation time to zero.

  while (true) {
    run_simulation();
  };
}

/* Generate a random vector with zero mean */
vector<float> generate_random_vector_zeromean(const int &length)
{
  // Generate the random vector
  vector<float> v(length, 0);
  for (int i = 0; i < length; i++) {
    v[i] = getrand_float(-0.5, 0.5);
  }

  // Adjust to zero mean
  vector<float> temp = v;
  for (int i = 0; i < length; i++) {
    v[i] = v[i] - vector_mean(temp);
  }

  return v;
}

void start_simulation(int argc, char *argv[])
{

  terminalinfo ti;

  // Extract number of agents from the argument
  if (argc <= 1) {
    ti.debug_msg("Please specify the amount of agents.\n\n");
    exit(1);
  } else {
    nagents = stoi(argv[1]);
    s.reserve(nagents); // Reserve spots for the expected amount of agents
  }


  if (argc <= 2) {
    ti.debug_msg("No nearest-neighbor rule specified. Assuming full connectivity. \n");
    knearest = nagents - 1;
  } else {
    knearest = stoi(argv[2]);
    if (knearest > (nagents - 1)) {
      ti.debug_msg("You can't have more nearest-neighbors that the number of observable agents. Quitting. \n");
      exit(1);
    }
  }

  // Generate random initial positions
  void randomgen_init();
  srand(time(NULL));
  vector<float> x0 = generate_random_vector_zeromean(nagents);
  vector<float> y0 = generate_random_vector_zeromean(nagents);

  // Set the model. This main should just spawn n agents at random positions/states.
  std::vector<std::thread> v;
  for (int i = 0; i < nagents; i++) {
    vector<float> states = { x0[i], y0[i], 0.0, 0.0, 0.0, 0.0 }; // Initial positions/states
    Particle p(i, states, 1.0 / param->simulation_updatefreq());
    s.push_back(&p);
  }

  // Launch agent threads
  for (int i = 0; i < nagents; i++) {
    thread agent(start_agent_simulation, i);
    agent.detach();
  }

  // TODO: Launch enironment threads
  // Gather enironment data (walls?!)
  // Launch a thread that handles all environment data
  // vector<float> wallpoints = {0, 0, 2, 2};

  simulation_start(); // Begin the simulation

}
#endif /*SIMULATION_H*/
