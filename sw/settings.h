#ifndef SETTINGS_H
#define SETTINGS_H

/**
 * Holds some higher level settings of the simulator
 *
 */

// Use SEQUENTIAL to launch robots one by one with the specified interval (in seconds), rather than all at once
// #define SEQUENTIAL 5

/**
 * Noise in relative sensing
 */
// TODO: Make runtime variable
#define NOISE_R 0 // STDEV of noise in range, used by omniscient_observer
#define NOISE_B 0 // STDEV of noise in bearing, used by omniscient_observer

#endif /*SETTINGS_H*/
