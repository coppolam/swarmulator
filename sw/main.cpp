// my first program in C++
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <vector>
#include <thread>         // std::thread
#include <mutex>

#include "agent.h"
#include "particle.h"
#include "main.h"
#include "animation.h"
#include "simulation.h"
#include "logger.h"
#include "omniscient_observer.h"

using namespace std;
int nagents;
vector<Particle> s;  // Set up a vector of relative position filters
int knearest;
float simulation_time;
float simtime_seconds;
mutex mtx;

/*
The main function launches separate threads that control independent
functions of the code. All threads are designed to be optional with the
exception of the simulation thread.
*/
int main(int argc, char* argv[])
{	
	/* Create a simulation thread */
	thread simulation (start_simulation, argc, argv);

    /* Create an animation thread */
	#ifdef ANIMATE
	thread animation (start_animation, argc, argv);
	#endif

	/* Create a logging thread to file log.txt*/
	#ifdef LOG
	thread logger (start_logger, argc, argv);
	#endif

	while(true){};  // Keep the program running

	return 0;
}