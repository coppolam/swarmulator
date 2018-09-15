#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <algorithm>
#include <stdint.h>
#include "omniscient_observer.h"

using namespace std;

/* 
This is a parent class for a controller. All controllers are children functions that must contain an implementation of get_velocity_command, which is declared virtual here.
*/
class Controller
{

public:
  Controller();  // Constructor
  ~Controller(); // Destructor

  float _ddes_x; // Desired equilibrium distance_x
  float _ddes_y; // Desired equilibrium distance_y
  float _kr;   // Repulsion gain
  float _ka;   // Attraction gain
  bool  saturation; // Define whether the controls are saturated
  float saturation_limits; // Define the saturation of the controls
  bool  moving; // Internal state of whether the robot is actively moving or not
  bool happy;
  // Set the saturation to true and set the saturation limits
  void set_saturation(const float &saturation_limits);
  
  // Function to saturate the commands
  void saturate(float &f);
  
  // Repulsion function. This is defined here as a basic for collision avoidance
  float f_repulsion(float u);

  // Virtual function to be implemented by child class that defines the controller, outputs desired v_x and v_y
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y) = 0;
};


#endif /*CONTROLLER_H*/
