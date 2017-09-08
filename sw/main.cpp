/*
	Swarmulator is a swarm simulation environment.
	Its design purpose is to be a simple testing platform to observe the emergent behavior of a group of agents. 
	To program specific behaviors, you can do so in the controller.cpp/controller.h file.

	To program specific behaviors, you can do so in the controller.cpp/controller.h class.

	Copyright Mario Coppola, 2017.
*/

// External includes
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <thread>         // std::thread
#include <mutex>

// Internal Includes
#include "agent.h"					// Agent
#include "particle.h"				
#include "main.h"					// Contains extern defines for global variables
#include "animation.h"				// Animation thread
#include "simulation.h"				// Simulation thread
#include "logger.h"					// Logger thread
#include "omniscient_observer.h"	// Class used to simulate sensing
#include "xmlreader.h"				
using namespace std;


#define ANIMATE // Activate animation thread
// #define whitebackground 	 // Use if you want a white background (can be nice for papers)
// #define LOG // Activate logger thread

// Only one of the following can work
// #define FORCED 	 // Forces to use a specific adjacency matrix as specified in "adjacencymatrix.txt" 
#define KNEAREST // Use a k-nearest topology according to the second argument
// #define ROGUE     // Would you like an agent to go rogue?
// #define rogueID 0 // ID of agent that goes rogue.

int nagents;			 // Number of agents in the simulation
vector<Particle> s;  	 // Set up a vector of relative position filters
int knearest;			 // knearest objects
mutex mtx;				 // Mutex needed to lock threads

// Default values
float simulation_time = 0;
float simtime_seconds = 0;
bool program_running = false;

// Animation
float simulation_updatefreq = 30;
float simulation_realtimefactor = 30;
int window_width  = 600;
int window_height = 600;
float scale = 0.3;
float mouse_drag_speed = 1.0;
float mouse_zoom_speed = 0.5;
float animation_updatefreq = 25;
int backgroundcolor; // Use if you want a white background (can be nice for papers)

// Logger
float logger_updatefreq = 1;

/*
	The main function launches separate threads that control independent
	functions of the code. All threads are designed to be optional with the
	exception of the simulation thread.
*/
int main(int argc, char* argv[])
{	
	program_running = true; // Program is running

	/* Read the parameters */
	XMLreader xmlrdr("conf/parameters.xml");
	xmlrdr.runthrough("simulation");
	xmlrdr.runthrough("animation");
	xmlrdr.runthrough("logger");

	/* Launch the simulation thread */
	thread simulation (start_simulation, argc, argv);

	/* Launch the animation thread */
	#ifdef ANIMATE
	thread animation (start_animation, argc, argv);
	#endif

	/* Launch the logging thread to file logs/log<date><time>.txt */
	#ifdef LOG
	thread logger (start_logger, argc, argv);
	#endif
	
	/* Keep the program running until terminated by a thread */
	while(program_running){};
	
	/* Exit */
	return 0;
}