#ifndef CONTROLLER_AGGREGATION_H
#define CONTROLLER_AGGREGATION_H

#include "controller.h"
#include "randomgenerator.h"
#include "trigonometry.h"

using namespace std;

/**
 * This controller handles attraction and velocity in North and East separately
 */
class controller_aggregation: public Controller
{
  vector<float> motion_p; // Probability of motion
  uint moving_timer; // Timer measuring how long a robot has been moving
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
public:

  /**
   * Construction. Controller_Aggregation is a child class of Controller.
   */
  controller_aggregation();

  /**
   * Implementation of method to get the commanded velocity
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_AGGREGATION_H*/
