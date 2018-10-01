#ifndef SETTINGS_H
#define SETTINGS_H

/***** Uncomment the main simulation threads and functions that run
 *     Note: this is on top of the main simulator thread
 *****/
#define ANIMATE // Activate animation thread
#define LOG     // Activate logger thread

// #define REMAIN_CONNECTED // Check that the swarm remains connected
// #define CHECK_HAPPY // Check whether the global goal is completed

/***** Select the controller to be used by uncommenting it ****/
// #define CONTROLLER Controller_Cartesian   // Basic x-y controller
// #define CONTROLLER Controller_Lattice   // Basic lattice controller
#define CONTROLLER Controller_Bearing_Shape  // Controller with Qf

/***** Select the agent type that you want to use by uncommenting it****/
#define AGENT Particle    // Accelerated particles
// #define AGENT Wheeled    // Wheeled vehicles

/***** Noise in relative sensing *****/
#define NOISE_R 0 // STDEV of noise in range
#define NOISE_B 0 // STDEV of noise in bearing

#endif /*SETTINGS_H*/
