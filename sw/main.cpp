/*
	Swarmulator is a swarm simulation.
	Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents. 
	To program specific behaviors, you can do so in the controller.cpp/controller.h file.

	Copyright Mario Coppola, 2017
*/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>         // std::thread
#include <mutex>

// Include other 
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "animation.h"				// Animation thread
#include "simulation.h"				// Simulation thread
#include "logger.h"					// Logger thread
#include "omniscient_observer.h"	

using namespace std;
int nagents;			 // Number of agents in the simulation
vector<Particle> s;  	 // Set up a vector of relative position filters
int knearest;			 // knearest objects
float simulation_time;	 // Time in the simulation
float simtime_seconds;	 // Global counter in real time
mutex mtx;				 // Mutex needed to lock threads

/*
The main function launches separate threads that control independent
functions of the code. All threads are designed to be optional with the
exception of the simulation thread.
*/
int main(int argc, char* argv[])
{	
	/* Create a simulation thread */
	thread simulation (start_simulation, argc, argv);

    /* Create an animation thread, if desired */
	#ifdef ANIMATE
	thread animation (start_animation, argc, argv);
	#endif

	/* Create a logging thread to file log.txt, if desired*/
	#ifdef LOG
	thread logger (start_logger, argc, argv);
	#endif

	while(true){};  // Keep the program running until terminated by a thread.

	return 0;
}