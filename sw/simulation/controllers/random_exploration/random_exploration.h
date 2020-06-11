#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "randomgenerator.h"

/**
 * Basic exploration behavior which randomly moves in an environment while avoiding neighbors.
 *
 */
class random_exploration: public Controller
{
  float v_x_old, v_y_old;

public:
  /**
   * @brief Construct a new random exploration object
   *
   */
  random_exploration();

  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*EXPLORATION_H*/
