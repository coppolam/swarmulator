/*
  Swarmulator is a swarm simulation environment.
  Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents.
  To program specific behaviors, you can do so in the controller.cpp/controller.h file.

  Copyright Mario Coppola, 2017-2018.
*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <thread>

// Internal Includes
#include "main.h"               /* Contains extern defines for global variables */
#include "simulation_thread.h"  /* Thread that handles the simulation */
#include "animation_thread.h"   /* Thread that handles animation */
#include "logger_thread.h"      /* Thread that handles the logger */

using namespace std;

/**
 * Global variables used throughout simulation
 */
uint nagents;                   /* Number of agents in the simulation */
vector<Agent *> s;              /* Set up the agents */
mutex mtx;                      /* Mutex needed to lock threads */
float realtimefactor;
int backgroundcolor;
float simulation_time = 0;
float simtime_seconds = 0;
float rangesensor     = 1.6;
bool program_running  = false;

/**
 * Parameters from the XML file
 */
unique_ptr<parameters_t> param(parameters("conf/parameters.xml", xml_schema::flags::dont_validate));

/**
 * The main function launches separate threads that control independent
 * functions of the code. All threads are designed to be optional with the
 * exception of the simulation thread.
 */
int main(int argc, char *argv[])
{
  program_running = true; // Program is running
  
  // Start simulation
  thread simulation(main_simulation_thread, argc, argv);
  simulation.detach();

#ifdef ANIMATE
  // Start animation
  thread animation(main_animation_thread);
  animation.detach();
#endif

#ifdef LOG
  // Start logger
  thread logger(main_logger_thread);
  logger.detach();
#endif

  // Keep the program running
  while (program_running) {};

  // Exit
  terminalinfo ti;
  ti.info_msg("Swarmulator exiting");
  return 0;
}