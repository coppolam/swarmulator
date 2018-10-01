#ifndef CONTROLLER_CARTESIAN_H
#define CONTROLLER_CARTESIAN_H

#include "controller.h"

using namespace std;

/*
 * This controller handles attraction and velocity in North and East separately
 */
class Controller_Cartesian: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver *o;

public:
  /**
   * Construction. Controller_Cartesian is a child class of Controller.
   */
  Controller_Cartesian(): Controller() {};

  /**
   * Attraction function at distance u
   */
  float f_attraction(float u);

  /**
   * Function to get the total attraction/repulsion velocity
   */ 
  float get_attraction_velocity(float u);

  /**
   * Implementation of method to get the commanded velocity
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_CARTESIAN_H*/