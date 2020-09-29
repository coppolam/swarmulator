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
#include "environment.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include "main.h"
#include AGENT_INCLUDE // from makefile

/**
 * Update the agent simulation
 * @param ID The ID of the agent/robot
 */
void run_agent_simulation_step(const int &ID)
{
  while (program_running) {
    bool sequential = (s.size() == nagents || simtime_seconds > 0.);
#ifdef SEQUENTIAL
    sequential = true;
#endif
    if (sequential) {
      mtx.lock_shared();
      std::vector<float> s_0 = s.at(ID)->state;
      std::vector<float> s_n = s.at(ID)->state_update(s_0); // State update
      mtx.unlock_shared();

      /****** Wall physics engine ********/
      // Check if hitting a wall
      std::vector<float> test = s_n;
      float r_temp, ang_temp, vx_temp, vy_temp;
      cart2polar(s_n[2], s_n[3], r_temp, ang_temp); // direction of velocity
      polar2cart(r_temp, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
      test[0] += vx_temp;
      test[1] += vy_temp;

      std::vector<float> laser_vals = s.at(ID)->laser_ranges;
      int idx = std::distance(laser_vals.begin(),std::min_element(laser_vals.begin(),laser_vals.end()));
  
      // if (!environment.sensor(ID, s_0, test, ang_temp)) { // No wall --> Update the dynamics
      // if (true) {
      if(laser_vals[idx] > s.at(ID)->laser_collision_threshold) {
        mtx.lock(); //sync
        s.at(ID)->state = s_n;
        mtx.unlock();
      } else { // Wall! --> Kill the dynamics
        mtx.lock(); //sync
        s.at(ID)->state[2] = 0.0; // v_x
        s.at(ID)->state[3] = 0.0; // v_y
        s.at(ID)->state[4] = 0.0; // a_x
        s.at(ID)->state[5] = 0.0; // a_y
        s.at(ID)->controller->moving = false; // Not moving
        mtx.unlock();
      }
      if (ID == 0) { // global clock = clock of first robot
        simtime_seconds += 1. / param->simulation_updatefreq();
        environment.loop();
      }
      if (param->simulation_realtimefactor() > 0) {
        /*** Sleep. Set param->simulation_realtimefactor()=0 in parameters.xml to avoid sleep and run at full speed! ***/
        int t_wait = (int)1e6 / (param->simulation_updatefreq() * param->simulation_realtimefactor());
        std::this_thread::sleep_for(std::chrono::microseconds(t_wait));
      }
    }
  }
}

/**
 * Generates new agent + simulation thread at given position x0 y0
 *
 * @param ID The ID of the agent/robot
 * @param x Initial position of the agent in x
 * @param y Initial position of the agent in y
 */
void create_new_agent(const int &ID, const std::vector<float> &states)
{
  mtx.lock();
  // Initiate a new agent
  s.push_back(new AGENT(ID, states, 1.0 / param->simulation_updatefreq()));
  mtx.unlock();

  // Info message
  std::stringstream ss;
  ss << "Robot " << ID << " initiated";
  terminalinfo::info_msg(ss.str());

  std::thread agent(run_agent_simulation_step, ID); // Initiate the thread that controls the agent
  agent.detach(); // Detach thread so that it runs independently
}
#endif /*AGENT_THREAD_H*/
