/*
 *
 * Swarmulator is a swarm simulation environment.
 * Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents.
 * To program specific behaviors, you can do so in the controller.cpp/controller.h file.
 *
 * Mario Coppola, 2017-2018.
 *
 */

/**
 * Include standard and thread libraries
 */
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <thread>

/**
 * Include top level threads
 */
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
float realtimefactor;           /* Real time factor of simulation */
float simulation_time = 0;      /* Initial simulation time (fake time) */
float simtime_seconds = 0;      /* Initial simulation time */
float rangesensor     = 1.7;    /* How far each robot can sense */
bool program_running  = false;  /* Program running, initiated false until the beginning */

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