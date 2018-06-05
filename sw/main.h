#ifndef MAIN_H
#define MAIN_H

#include <mutex>
#include <vector>
#include "particle.h"
#include "settings.h"
#include "parameters.hxx"

// Simulation variables
extern int nagents; // Number of agents in the swarm
extern int knearest;  // Knearest neighbours
extern std::vector<Particle> s; // Set up a vector of particles
// extern std::vector<Wall> w; // Set up a vector of walls
extern float simulation_time; // Simulation time (our time)
extern float simtime_seconds; // Adjusted simulation time (time according to simulation)
extern std::mutex mtx; // Mutex for critical section
extern bool program_running; // True if the program is (or should be) running. If false the program shuts down.
extern unique_ptr<parameters_t> param; // XML parameters from conf file
extern int window_width, window_height;

#endif /*MAIN_H*/