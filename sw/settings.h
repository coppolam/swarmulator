#ifndef SETTINGS_H
#define SETTINGS_H

/***** Uncomment the main simulation threads and functions that run
 *     Note: this is on top of the main simulator thread
 *****/
#define ANIMATE // Activate animation thread
#define LOG // Activate logger thread

// #define REMAIN_CONNECTED // Check that the swarm remains connected
// #define CHECK_HAPPY

/***** Select the controller to be used ****/
// #define CONTROLLER Controller_Cartesian
#define CONTROLLER Controller_Bearing_Shape
// #define CONTROLLER Controller_Lattice
// #define CONTROLLER Controller_Aggregate
// #define CONTROLLER Controller_Keep_Aggregate

/***** Select the agent type that you want to use ****/
#define AGENT Particle
// #define AGENT Wheeled

#define NOISE_R 0
#define NOISE_B 0

#endif /*SETTINGS_H*/
