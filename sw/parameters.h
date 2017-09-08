#ifndef PARAMETERS_H
#define PARAMETERS_H
/* Put all defines in here. Eventually this will be moved to an xml file */

// Simulation Parameters
/*#define simulation_updatefreq 30.0 	  // (Hz) Defines the simulation time step
#define simulation_realtimefactor 30.0 // Real time factor of simulation
*/
// Animation Parameters
#define ANIMATE // Activate animation thread
// #define window_width  600//1920	 // Native HD window width  (px)
// #define window_height 600//1080	 // Native HD window height (px) 
// #define scale 0.3			 // Scale of agent size
// #define mouse_drag_speed 1.0 // Speed of mouse to drag animation
// #define mouse_zoom_speed 0.5 // Zoom speed of scroll wheel
// #define animation_updatefreq  25.0  // Animation fps
// #define whitebackground 	 // Use if you want a white background (can be nice for papers)

#define LOG // Activate logger thread
#define logger_updatefreq 1	 // Hz (with respect to simulation time!)

// Only one of the following works
// #define FORCED 	 // Forces to use a specific adjacency matrix as specified in "adjacencymatrix.txt" 
#define KNEAREST // Use a k-nearest topology according to the second argument

// Bonus functions
// #define ROGUE     // Would you like an agent to go rogue?
// #define rogueID 0 // ID of agent that goes rogue.

// TODO: Add environmet
// #define ENVIRONMENT
// #define INFO

#endif /*PARAMETERS_H*/