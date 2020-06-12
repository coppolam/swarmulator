# forage

This controller makes the agents look for food in the environment and return it to a home beacon.

The home beacon is placed at (0,0) by default.
The environment has 100 food items placed in it by default.
Please check the Environment class (`sw/simulation/environment.cpp` to modify these parameters).

IMPORTANT NOTE: In the standar version, the food will only show up if the fitness function used is `food`, as indicated int he runtime parameters.

Designed to work with AGENT=particle_oriented.