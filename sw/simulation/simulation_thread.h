#ifndef SIMULATION_THREAD_H
#define SIMULATION_THREAD_H

#include <numeric>
#include <functional>
#include <cctype>
#include <algorithm>
#include <thread>

#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include "agent_thread.h"
#include "drawingparams.h"

/*
 * Calculate the mean of all elements in a vector 
 */
// TODO: Move to math
float vector_mean(const vector<float> &v)
{
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  return sum / v.size();
}

/* 
 * Calculate the standard deviation of all elements in a vector
 */
// TODO: Move to math
float vector_std(const vector<float> &v)
{
  vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), vector_mean(v))
                );
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  return sqrt(sq_sum / v.size());
}

/*
 * Generate a random vector with zero mean
 */
// TODO: Move to math
vector<float> generate_random_vector_zeromean(const int &length)
{
  // Generate the random vector
  vector<float> v(length, 0);
  for (uint8_t i = 0; i < length; i++) {
    v[i] = getrand_float(-0.5, 0.5);
  }

  // Adjust to zero mean
  vector<float> temp = v;
  for (uint8_t i = 0; i < length; i++)
  {
    v[i] = v[i] - vector_mean(temp);
  }

  return v;
}

/*
 * Extract the number of agents from the argument list.
 * Else, return an error.
 */
void get_number_of_agents(int argc, char *argv[])
{
  terminalinfo ti;
  if (argc <= 1) {
    ti.info_msg("Please specify the number of agents.");
    program_running = false;
  } else {
    nagents = stoi(argv[1]);
  }
}

/*
 * This function initiates the simulation.
 * All agents in the beginning initiate randomly with a mean position around the (0,0) point.
 * Once the vector of agents is created, each agent is launched in a separate thread.
 */

void main_simulation_thread(int argc, char *argv[])
{
  terminalinfo ti;
  ti.info_msg("Simulation started.");

  // Read the number of agents from the argument input
  get_number_of_agents(argc, argv);

  // Generate the random initial positions with (0,0) mean
  randomgen_init();
  srand(time(NULL));
  vector<float> x0 = generate_random_vector_zeromean(nagents);
  vector<float> y0 = generate_random_vector_zeromean(nagents);

  // Generate the agent models
  for (uint8_t ID = 0; ID < nagents; ID++) {
    // Initial positions/states
    vector<float> states = { x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0 };
    // The data of the swarm is stored in a vector
    // AGENT is a stand-in for the agent of choice. This is selected in setting.h
    s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  }

  // Launch agent threads to simulate each agent independetly
  for (uint8_t ID = 0; ID < nagents; ID++) {
    thread agent(start_agent_simulation, ID);
    agent.detach();
  }

  // Keep global clock running.
  // This is only used by the animation and the logger.
  // The robots operate by their own clock)
  while (program_running) {
    if (!paused)
    {
      int t_wait = (int)1000000.0 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
      this_thread::sleep_for(chrono::microseconds(t_wait));
      simulation_time = t_wait;
      simtime_seconds += param->simulation_realtimefactor() * simulation_time / 1000000.0;
    }
  };

}
#endif /*SIMULATION_THREAD_H*/
