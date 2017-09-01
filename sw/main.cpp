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
#include <pugixml/pugixml.hpp>

// Internal Includes
#include "agent.h"
#include "particle.h"				
#include "main.h"					// Contains extern defines for global variables
#include "animation.h"				// Animation thread
#include "simulation.h"				// Simulation thread
#include "logger.h"					// Logger thread
#include "omniscient_observer.h"	// Class used to simulate sensing

using namespace std;

int nagents;			 // Number of agents in the simulation
vector<Particle> s;  	 // Set up a vector of relative position filters
int knearest;			 // knearest objects
float simulation_time;	 // Time in the simulation
float simtime_seconds;	 // Global counter in real time
mutex mtx;				 // Mutex needed to lock threads
bool program_running;    // True if the program is running, turns false when the program is shut down

/*
	The main function launches separate threads that control independent
	functions of the code. All threads are designed to be optional with the
	exception of the simulation thread.
*/
int main(int argc, char* argv[])
{	
	program_running = true; // Program is running

	// Load XML file
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file("conf/parameters.xml");
	if (result.status) // False (0) = No errors
	{
		debug_msg ("Could not load parameters file.");
		return 0;
	}
		
	pugi::xml_node node_sim = doc.child("simulation");
	doc.child("simulation").attribute("name").value();
	for (pugi::xml_node node_sim = node_sim.first_child(); node_sim; node_sim = node_sim.next_sibling())
    {
        std::cout << node_sim.name() << std::endl;

        for (pugi::xml_attribute attr = node_sim.first_attribute(); attr; attr = attr.next_attribute())
        {
            std::cout << " " << attr.name() << "=" << attr.value() << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

	// std::cout << "Load result: " <<  << ", mesh name: " << doc.child("mesh").attribute("name").value() << std::endl;

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