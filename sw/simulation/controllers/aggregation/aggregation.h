#ifndef AGGREGATION_H
#define AGGREGATION_H

#include "controller.h"
#include "randomgenerator.h"
#include "trigonometry.h"

/**
 * This controller handles attraction and velocity in North and East separately
 */
class aggregation: public Controller
{
  std::vector<float> motion_p; // Probability of motion
  uint moving_timer; // Timer measuring how long a robot has been moving
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;

public:

  /**
   * Construction. Controller_Aggregation is a child class of Controller.
   */
  aggregation();

  /**
   * Implementation of method to get the commanded velocity
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*AGGREGATION_H*/
