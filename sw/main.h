// Simulation variables
#ifndef MAIN_H
#define MAIN_H

#include <mutex>
#include <vector>
#include "particle.h"

extern int nagents;       // Number of agents in the swarm
extern int knearest;      // Knearest neighbours
extern std::vector<Particle> s; // Set up a vector of relative position filters
extern float simulation_time; // Simulation time (our time)
extern float simtime_seconds;   // Adjusted simulation time (time according to simulation)
extern std::mutex mtx;          // mutex for critical section
extern bool program_running;

// Simulation parameters
extern float simulation_updatefreq;   // (Hz) Defines the simulation time step
extern float simulation_realtimefactor; // Real time factor of simulation

// Animation parameters
extern int window_width;        // Window width  (px)
extern int window_height;       // Window height (px)
extern float scale;             // Scale of agent size
extern float mouse_drag_speed;  // Speed of mouse to drag animation
extern float mouse_zoom_speed;  // Zoom speed of scroll wheel
extern float animation_updatefreq;  // Animation fps
extern bool visible_centroid;    // Centroid of swarm is visible
extern int backgroundcolor;     // Use if you want a white background (can be nice for papers)

extern float logger_updatefreq;   // Use if you want a white background (can be nice for papers)

#define ANIMATE // Activate animation thread
// #define whitebackground   // Use if you want a white background (can be nice for papers)
// #define LOG // Activate logger thread

// Only one of the following can work
// #define FORCED    // Forces to use a specific adjacency matrix as specified in "adjacencymatrix.txt"
#define KNEAREST // Use a k-nearest topology according to the second argument
// #define ROGUE     // Would you like an agent to go rogue?
// #define rogueID 0 // ID of agent that goes rogue.

#endif /*MAIN_H*/
