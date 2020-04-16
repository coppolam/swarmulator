#ifndef SETTINGS_H
#define SETTINGS_H

/**
 * Uncomment the main simulation threads and functions that run
 * Note: this is on top of the main simulator thread
 */

// Options
// #define SEQUENTIAL 5
#define ESTIMATOR
// #define REMAIN_CONNECTED // Check that the swarm remains connected (only if logging)
// #define CHECK_HAPPY // Check whether the global goal is completed  (only if logging)

/**
 * Select the controller to be used by uncommenting it
 */
// #define CONTROLLER controller_lattice  // Basic lattice controller
// #define CONTROLLER controller_pattern  // Controller with Qf

// #define CONTROLLER ndi_follower  // NDI follower, needs AGENT particle_oriented, to be defined below!
// #define COMMAND_LOCAL 1  // use COMMAND_LOCAL for local commands

// #define CONTROLLER behavior_tree // Controller with behvaior_tree_aggregation
// #define CONTROLLER behavior_tree_wheeled // Controller with Qf

#define CONTROLLER controller_aggregation // Aggregation controller (needs walls!)

/**
 * Select the agent type that you want to use by uncommenting it
 */
#define AGENT particle    // Accelerated particles
// #define AGENT particle_oriented    // Accelerated particle with orientation
// #define AGENT wheeled    // Wheeled robot

/**
 * Noise in relative sensing
 */
// TODO: Make runtime variable
#define NOISE_R 0 // STDEV of noise in range
#define NOISE_B 0 // STDEV of noise in bearing

#define FIFO_FILE "/tmp/swarmulator"

#endif /*SETTINGS_H*/
