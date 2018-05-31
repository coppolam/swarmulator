#ifndef AGENTTHREAD_H
#define AGENTTHREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>

void run_agent_simulation_step(int id)
{
  // The mutex ensures that while the positions are updated by the simulation thread, other threads are not trying to access the data.
  if (s.size()-nagents==0) {
  mtx.lock();
  s[id].update_position();
  mtx.unlock();
  }

  // Increase time to the next timestep
  int t_wait = (int) 1000000.0 * (1.0 / (simulation_updatefreq * simulation_realtimefactor));
  this_thread::sleep_for(chrono::microseconds(t_wait));
}

void run_agent_simulation(int id)
{
  while(true) {
    run_agent_simulation_step(id);
  }
}

void start_agent_simulation(int id)
{
  std::cout << "Agent" << id << "started"<< endl;
  run_agent_simulation(id);
};
#endif /*AGENTTHREAD_H*/
