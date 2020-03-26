#ifndef CONTROLLER_AGGREGATION_H
#define CONTROLLER_AGGREGATION_H

#include "controller.h"
#include "randomgenerator.h"
#include "trigonometry.h"

using namespace std;

/*
 * This controller handles attraction and velocity in North and East separately
 */
class controller_aggregation: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;

public:
  bool moving; // Robot sets it if it is moving.
  float v_x_ref, v_y_ref;
  vector<float> motion_p; // Probability of motion
  uint moving_timer; // Timer measuring how long a robot has been moving
  random_generator rg; // Random generator

  /**
   * Construction. Controller_Aggregation is a child class of Controller.
   */
  controller_aggregation();

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

  void wall_avoidance(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_AGGREGATION_H*/
