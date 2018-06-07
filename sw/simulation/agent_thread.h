#ifndef AGENTTHREAD_H
#define AGENTTHREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include "particle.h"

void run_agent_simulation_step(const int &id)
{
  // The mutex ensures that while the positions are updated by the simulation thread, other threads are not trying to access the data.
  if ((uint8_t)s.size() <= nagents)
  {
    cout << id;
    mtx.lock();
    cout << " " << s[id]->get_position(1) << " " ;
    s.at(id)->update_position();
    // cout << " " << s[id]->get_position(1) << " " << endl;
    mtx.unlock();
  }

  // Increase time to the next timestep
  int t_wait = (int)1000000.0 * (1.0 / (param->simulation_updatefreq() * param->simulation_realtimefactor()));
  this_thread::sleep_for(chrono::microseconds(t_wait));
}

void start_agent_simulation(int id)
{
  std::cout << "Agent" << id << "started" << endl;
  while (true)
  {
    run_agent_simulation_step(id);
  }
};
#endif /*AGENTTHREAD_H*/
