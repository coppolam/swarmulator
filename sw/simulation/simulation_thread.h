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
#include "settings.h"

/**
 * Extract the number of agents from the argument list.
 * Else, return an error.
 * 
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
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

/**
 * This function initiates the simulation.
 * All agents in the beginning initiate randomly with a mean position around the (0,0) point.
 * Once the vector of agents is created, each agent is launched in a separate thread.
 * 
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void main_simulation_thread(int argc, char *argv[])
{
  terminalinfo ti;
  ti.info_msg("Simulation started.");

  // Read the number of agents from the argument input
  get_number_of_agents(argc, argv);

  // Generate the random initial positions with (0,0) mean and 0.5 standard deviation
  random_generator rg;
  float spread = 0.5; // default
#ifdef ARENAWALLS
  spread = (float)ARENAWALLS / (float)3.0;
#endif

  vector<float> x0 = rg.uniform_float_vector(nagents, -spread, spread);
  vector<float> y0 = rg.uniform_float_vector(nagents, -spread, spread);
  vector<float> t0 = rg.uniform_float_vector(nagents, -M_PI, M_PI);

  // Generate the agent models
  for (uint8_t ID = 0; ID < nagents; ID++) {
    // Initial positions/states
    vector<float> states = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
    // The data of the swarm is stored in a vector
    // AGENT is a stand-in for the agent of choice. This is selected in setting.h
    s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  }

  // Launch agent threads to simulate each agent independently
  for (uint8_t ID = 0; ID < nagents; ID++) {
    thread agent(start_agent_simulation, ID);
    agent.detach();
  }

  // Keep global clock running.
  // This is only used by the animation and the logger.
  // The robots operate by their own detached thread clock.
  while (program_running) {
    if (!paused) {
      int t_wait = (int)1e9 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
      this_thread::sleep_for(chrono::nanoseconds(t_wait));
      simulation_time = t_wait;
      mtx.lock(); // Lock mutex to update global clock thread in relative sync
      simtime_seconds += param->simulation_realtimefactor() * simulation_time / 1e9;
      mtx.unlock();
#ifdef LOGTIME
      if (simtime_seconds > LOGTIME) { // Quit after a certain amount of time
        program_running = false;
      }
#endif
    }
  };

}
#endif /*SIMULATION_THREAD_H*/
