/*
  Swarmulator is a swarm simulation environment.
  Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents.
  To program specific behaviors, you can do so in the controller.cpp/controller.h file.

  Copyright Mario Coppola, 2017-2018.
*/

// External includes
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread> // std::thread
#include <mutex>

// Internal Includes
#include "main.h" // Contains extern defines for global variables
#include "animation_thread.h"
#include "simulation_thread.h"
#include "logger_thread.h"

using namespace std;

// Simulation default values
uint nagents; // Number of agents in the simulation
vector<Agent *> s; // Set up a vector of relative position filters
// vector<Wall> w; // Set up a vector of walls
uint knearest; // knearest objects
mutex mtx; // Mutex needed to lock threads
float simulation_time = 0;
float simtime_seconds = 0;
bool program_running = false;
uint window_width, window_height;
float realtimefactor;
float rangesensor = 1.6;
int backgroundcolor; // Use if you want a white background (can be nice for papers)

// Parameters XML parser
unique_ptr<parameters_t> param(parameters("conf/parameters.xml", xml_schema::flags::dont_validate));

/*
  The main function launches separate threads that control independent
  functions of the code. All threads are designed to be optional with the
  exception of the simulation thread.
*/
int main(int argc, char *argv[])
{
  program_running = true; // Program is running
  window_height = param->window_height();
  window_width = param->window_width();

  thread simulation(main_simulation_thread, argc, argv);
  simulation.detach();

#ifdef ANIMATE
  thread animation(main_animation_thread);
  animation.detach();
#endif

#ifdef LOG
  thread logger(main_logger_thread);
  logger.detach();
#endif

  while (program_running) {}; // Keep the program running
  cout << "Terminating swarmulator" << endl;
  return 0;
}