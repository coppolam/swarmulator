#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "randomgenerator.h"

#define COMMAND_LOCAL 1

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
  virtual void animation(const uint16_t ID);
};

#endif /*EXPLORATION_H*/
