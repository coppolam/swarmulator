#ifndef AGENTTHREAD_H
#define AGENTTHREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include <chrono>

#include "settings.h"
#include "randomgenerator.h"
#include "includes_agents.h" // Include all agents here
#include "environment.h"
#include "terminalinfo.h"

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
  vector<float> s_n = s.at(ID)->state_update(s.at(ID)->state);
  vector<float> test = s_n;
  test[0] += 20 * s_n[2]; // Gross prediction margin ahead as safety margin
  test[1] += 20 * s_n[3];
  if (!environment.sensor(ID, s.at(ID)->state, test)) {
    mtx.lock();
    s.at(ID)->state = s_n;
    mtx.unlock();
  } else {
    // TODO: Provide different options?!
    mtx.lock();
    s.at(ID)->state[2] = 0.0;
    s.at(ID)->state[3] = 0.0;
    s.at(ID)->state[4] = 0.0;
    s.at(ID)->state[5] = 0.0;
    mtx.unlock();
  }
  auto end = chrono::steady_clock::now();
  auto duration = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
#ifdef LOGGER
  char a[20];
  sprintf(a, "%ld\n", test);
  logfile << a;
#endif
  // Wait according to defined frequency (subtract execution time)
  int t_wait = (int)1e9 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor())) - duration;
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
