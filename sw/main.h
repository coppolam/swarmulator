// Simulation variables
#ifndef MAIN_H
#define MAIN_H

extern int nagents;	 			// Number of agents in the swarm
extern int knearest;			// Knearest neighbours
extern vector<Particle> s;  	// Set up a vector of relative position filters
extern float simulation_time;	// Simulation time (our time)
extern float simtime_seconds;   // Adjusted simulation time (time according to simulation)
extern std::mutex mtx;          // mutex for critical section

#endif /*MAIN_H*/
