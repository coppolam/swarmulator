/*
	Swarmulator is a swarm simulation.
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
#include "agent.h"
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
#define LOG // Activate logger thread

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
float simulation_time = 0;	 // Time in the simulation
float simtime_seconds = 0;	 // Global counter in real time
bool program_running = false; // True if the program is running, turns false when the program is shut down
float simulation_updatefreq = 30; // (Hz) Defines the simulation time step
float simulation_realtimefactor = 30; // Real time factor of simulation
int window_width = 600;  //1920	 // Native HD window width  (px)
int window_height = 600; //1080	 // Native HD window height (px) 
float scale = 0.3;       // Scale of agent size
float mouse_drag_speed = 1.0;  // Speed of mouse to drag animation
float mouse_zoom_speed = 0.5; // Zoom speed of scroll wheel
float animation_updatefreq = 25;  // Animation fps
int backgroundcolor; // Use if you want a white background (can be nice for papers)
float logger_updatefreq = 1;

/*
	The main function launches separate threads that control independent
	functions of the code. All threads are designed to be optional with the
	exception of the simulation thread.
*/
int main(int argc, char* argv[])
{	
	program_running = true; // Program is running

	XMLreader xmlrdr("conf/parameters.xml");
	xmlrdr.runthrough("simulation");
	xmlrdr.runthrough("animation");
	xmlrdr.runthrough("logger");

	/* Create a simulation thread */
	thread simulation (start_simulation, argc, argv);

	/* Create an animation thread, if desired */
	#ifdef ANIMATE
	thread animation (start_animation, argc, argv);
	#endif

	/* Create a logging thread to file log.txt, if desired */
	#ifdef LOG
	thread logger (start_logger, argc, argv);
	#endif
	
	/* Keep the program running until terminated by a thread */
	while(program_running){};  
	
	/* Exit */
	return 0;
}