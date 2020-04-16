#ifndef AGENT_THREAD_H
#define AGENT_THREAD_H

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
#include "auxiliary.h"

/**
 * Update the agent simulation
 *
 * @param ID The ID of the agent/robot
 * @param logfile The file ID of the log
 */
void run_agent_simulation_step(const int &ID, ofstream &logfile)
{
  // Update the position of the agent in the simulation
  // auto start = chrono::steady_clock::now();
  mtx.lock(); // Lock mutex to avoid conflicts
  vector<float> s_n = s.at(ID)->state_update(s.at(ID)->state); // State update
  mtx.unlock();
  /****** Wall physics engine ********/
  // Check if hitting a wall
  vector<float> test = s_n;
  float r_temp, ang_temp, vx_temp, vy_temp;
  cart2polar(s_n[2], s_n[3], r_temp, ang_temp); // direction of velocity
  polar2cart(2, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  test[0] += vx_temp;
  test[1] += vy_temp;
  if (!environment.sensor(ID, s.at(ID)->state, test, ang_temp)) { // No wall --> Update the dynamics
    mtx.lock(); // Lock mutex to avoid conflicts
    s.at(ID)->state = s_n;
    mtx.unlock();
  } else { // Wall! --> Kill the dynamics
    mtx.lock();
    s.at(ID)->state[2] = 0.0; // v_x
    s.at(ID)->state[3] = 0.0; // v_y
    s.at(ID)->state[4] = 0.0; // a_x
    s.at(ID)->state[5] = 0.0; // a_y
    s.at(ID)->controller->moving = false; // Not moving
    mtx.unlock();
  }
  /**********************************/
  // auto end = chrono::steady_clock::now();
  // auto duration = chrono::duration_cast<chrono::microseconds>(end - start).count();

  // Wait according to defined frequency (subtract execution time)
  int t_wait = (int)1e6 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
  this_thread::sleep_for(chrono::microseconds(t_wait));
}

/**
 * Start the simulation of an agent
 *
 * @param ID The ID of the agent/robot
 */
void start_agent_simulation(const int &ID)
{
  // Info message
  stringstream ss;
  ss << "Robot " << ID << " initiated";
  terminalinfo::info_msg(ss.str());

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
void create_new_agent(const int &ID, const vector<float> &states)
{
  // Initiate a new agent
  mtx.lock();
  s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  mtx.unlock();

  thread agent(start_agent_simulation, ID); // Initiate the thread that controls the agent
  agent.detach(); // Detach thread so that it runs independently
}
#endif /*AGENT_THREAD_H*/
