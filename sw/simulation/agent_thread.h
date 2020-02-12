#ifndef AGENTTHREAD_H
#define AGENTTHREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include "settings.h"
#include "randomgenerator.h"
#include <chrono>

// Include all agents here
#include "includes_agents.h"


/**
 * Update the agent simulation
 * 
 * @param ID The ID of the agent/robot
 * @param logfile The file ID of the log
 */
void run_agent_simulation_step(const int &ID, ofstream &logfile)
{
  // Update the position of the agent in the simulation
  // Lock mutex to avoid conflicts
  auto start = chrono::steady_clock::now();
  mtx.lock();
  s.at(ID)->state_update();
  auto end = chrono::steady_clock::now();
  auto test = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
  char a[20];
  sprintf(a, "%ld\n", test);
  logfile << a;
  mtx.unlock();

  // Wait according to define frequency
  int t_wait = (int)1000000000.0 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor())) - test;
  this_thread::sleep_for(chrono::nanoseconds(t_wait));
}

/**
 * Start the simulation of an agent
 * 
 * @param ID The ID of the agent/robot
 */
void start_agent_simulation(int ID)
{
  // Info message
  terminalinfo ti;
  stringstream ss;
  ss << "Robot " << ID << " initiated";
  ti.info_msg(ss.str());

  char filename[100];
  sprintf(filename, "logs/evaluation_time/timelog_%d.txt", nagents);
  ofstream file;
  file.open(filename);

  // Run the new robot
  while (program_running) {
    run_agent_simulation_step(ID, file);
  }
};

/**
 * Generates new agent + simulation thread at given position x0 y0
 * 
 * @param ID The ID of the agent/robot
 * @param x Initial position of the agent in x
 * @param y Initial position of the agent in y
 */
void create_new_agent(int ID, float x0, float y0)
{
  // Initiate a new agent at the given position
  random_generator rg;
  vector<float> states = {x0, y0, 0.0, 0.0, 0.0, 0.0, rg.uniform_float(-M_PI, M_PI), 0.0}; // Initial positions/states
  s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  nagents++; // Increase agent counter

  // Wait a bit before animating the new agent
  this_thread::sleep_for(chrono::microseconds(1000));

  // Initate and detach the threads
  thread agent(start_agent_simulation, ID); // Initiate the thread that controls the agent
  agent.detach(); // Detach thread so that it runs independently
}
#endif /*AGENTTHREAD_H*/
