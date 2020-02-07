#ifndef CONTROLLER_CARTESIAN_H
#define CONTROLLER_CARTESIAN_H

#include "controller.h"
#include "randomgenerator.h"
#include "trigonometry.h"

using namespace std;

/*
 * This controller handles attraction and velocity in North and East separately
 */
class Controller_Aggregation: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;

public:
  bool moving; // Robot sets it if it is moving.
  float v_x_ref, v_y_ref, ang;
  vector<float> motion_p; // Probability of motion
  int moving_timer; // Timer measuring how long a robot has been moving
  int walltimer; // Timer to set the time for a wall avoidance maneuver
  random_generator rg; // Random generator

  /**
   * Construction. Controller_Aggregation is a child class of Controller.
   */
  Controller_Aggregation() : Controller()
  {
    moving = false;
    v_x_ref = rg.gaussian_float(0.0, 1.0);
    v_y_ref = rg.gaussian_float(0.0, 1.0);
    float r; //temp
    cart2polar(v_x_ref, v_y_ref, r, ang);
    // motion_p = {P1, P2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    motion_p = {0.991355, 0.984845, 0.007304, 0.000783, 0.004238, 0.001033, 0.007088};

    moving_timer = 0;
    walltimer = 0;
  };

  /**
   * Attraction function at distance u
   */
  float f_attraction(float u);

  /**
   * Function to get the total attraction/repulsion velocity
   */
  float get_attraction_velocity(float u);

  /**
   * Function to give the commands for robots to stay in a lattice
   */
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);

  /**
   * Implementation of method to get the commanded velocity
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_CARTESIAN_H*/
