#ifndef SETTINGS_H
#define SETTINGS_H

/***** Uncomment the main simulation threads and functions that run
 *     Note: this is on top of the main simulator thread
 *****/
#define ANIMATE // Activate animation thread
// #define LOG     // Activate logger thread
#define MAX_TIME 100 // If set, swarmulator will quite automatically after 100 (or however many) simulated seconds. Requires logger.

// #define REMAIN_CONNECTED // Check that the swarm remains connected (only if logging)
// #define CHECK_HAPPY // Check whether the global goal is completed  (only if logging)

/***** Select the controller to be used by uncommenting it ****/
// #define CONTROLLER Controller_Aggregation  // Aggregation controller
#define ARENAWALLS 30  // Define the area for the aggreagation controller!

// #define CONTROLLER Controller_Lattice  // Basic lattice controller
// #define CONTROLLER Controller_Pattern  // Controller with Qf

// #define CONTROLLER ndi_follower  // NDI follower, needs AGENT Wheeled, to be defined below!
// #define COMMAND_LOCAL 1  // use COMMAND_LOCAL for local commands

#define CONTROLLER behavior_tree  // Controller with Qf

/***** Select the agent type that you want to use by uncommenting it****/
#define AGENT Particle    // Accelerated particles
// #define AGENT Wheeled    // Wheeled vehicles

/***** Noise in relative sensing *****/
#define NOISE_R 0 // STDEV of noise in range
#define NOISE_B 0 // STDEV of noise in bearing

#endif /*SETTINGS_H*/
