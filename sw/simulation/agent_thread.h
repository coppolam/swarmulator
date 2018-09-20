#ifndef AGENTTHREAD_H
#define AGENTTHREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include "settings.h"

// Include all agents here
#include "includes_agents.h"

// Update the agent simulation
void run_agent_simulation_step(const int &id)
{
  // Update the position of the agent in the simulation
  // Lock mutex to avoid conflicts
  mtx.lock();
  s.at(id)->state_update();
  mtx.unlock();

  // Wait according to define frequency
  int t_wait = (int)1000000.0 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
  this_thread::sleep_for(chrono::microseconds(t_wait));
}

// Start the simulation of an agent
void start_agent_simulation(int id)
{
  // Info message
  terminalinfo ti;
  stringstream ss;
  ss << "Robot " << id << " intiated";
  ti.info_msg(ss.str());

  // Run the new robot
  while (program_running) {
    run_agent_simulation_step(id);
  }
};

// Generates new agent + simulation thread at given position x0 y0
void create_new_agent(int ID, float x0, float y0)
{
  // Initiate a new agent at the given position
  vector<float> states = {x0, y0, 0.0, 0.0, 0.0, 0.0}; // Initial positions/states
  s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  nagents++;

  // Wait a bit before animating the new agent
  this_thread::sleep_for(chrono::microseconds(1000));

  // Initate and detach the threads
  thread agent(start_agent_simulation, ID); // Initiate the thread that controls the agent
  agent.detach(); // Detach thread so that it runs independently
}
#endif /*AGENTTHREAD_H*/
