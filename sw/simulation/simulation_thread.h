#ifndef SIMULATION_THREAD_H
#define SIMULATION_THREAD_H

#include <numeric>
#include <functional>
#include <cctype>
#include <algorithm>
#include <thread>

#include <cstdlib> // system, NULL, EXIT_FAILURE
#include <iostream>
#include <sstream> // std::stringstream, std::stringbuf
#include <future>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include "agent_thread.h"
#include "drawingparams.h"
#include "settings.h"
#include "environment.h"
#include "fitness_functions.h"

#define FIFO_FILE "/tmp/swarmulator"

/**
 * Select a fitness function, or use your own if you want.
 * The file "fitness_functions.h" is used to store fitness functions.
 *
 * @return float fitness
 */
float evaluate_fitness()
{
  float f;
  // f = mean_dist_to_one_neighbor(0);
  f = mean_number_of_neighbors();
  // f = mean_dist_to_neighbors();
  // connectivity_check(f);

  return f;
}

/**
 * @brief Send out a FIFO message to interfact with external programs.
 *
 * @param fd FIFO file ID, initiated in the beginning of the thread
 * @return int
 */
int send_fifo(int fd)
{
  float fitness = evaluate_fitness();
  uint8_t size = 8;
  char msg[size];
  sprintf(msg, "%f", fitness);
  return write(fd, (char *)msg, size * sizeof(char));
}

/**
 * Extract the number of agents from the argument list.
 * Else, return an error.
 *
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void get_number_of_agents(int argc, char *argv[])
{
  if (argc <= 1) {
    terminalinfo::error_msg("Please specify the number of agents.");
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
  // Create and open the FIFO pipe to communicate/interface with external programs, if needed.
  // By default, it is used to send the fitness.
  char const *bt_fifo_write = FIFO_FILE;
  int fd_write;
  if (access(bt_fifo_write, F_OK) == -1) {
    mkfifo(bt_fifo_write, 0666);
  }
  fd_write = open(bt_fifo_write, O_RDWR | O_NONBLOCK);

  terminalinfo::info_msg("Simulation started.");

  // Read the number of agents from the argument input
  get_number_of_agents(argc, argv);

  // Generate the random initial positions with (0,0) mean and 0.5 standard deviation
  random_generator rg;
  float spread = environment.limits(); // default // TODO: Spread randomly within an arbitray arena
  vector<float> x0 = rg.uniform_float_vector(nagents, -spread, spread);
  vector<float> y0 = rg.uniform_float_vector(nagents, -spread, spread);
  vector<float> t0 = rg.uniform_float_vector(nagents, -M_PI, M_PI);

  // Generate the agent models
  for (uint8_t ID = 0; ID < nagents; ID++) {
    // Initial positions/states
    vector<float> states = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
    // The data of the swarm is stored in a vector
    // AGENT is a stand-in for the agent of choice. This is selected in setting.h
    s.push_back(new AGENT(ID, states, 1.0 / (param->simulation_updatefreq())));
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
      // Runtime finish evolution
      if (param->time_limit() > 0.0) {
        if (simtime_seconds > param->time_limit()) { // Quit after a certain amount of time
          send_fifo(fd_write);
          program_running = false;
        }
      }
    }
  };

}
#endif /*SIMULATION_THREAD_H*/
