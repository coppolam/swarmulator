#ifndef PYTORCH_H
#define PYTORCH_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

class pytorch: public Controller
{
public:
  /**
   * @brief Construct a new pytorch controller
   *
   */
  pytorch(): Controller() {};

  /**
   * @brief Get the velocity command object
   *
   * @param ID
   * @param v_x
   * @param v_y
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);

  /**
   * @brief Animation
   *
   * @param ID
   */
  virtual void animation(const uint16_t ID);
};

#endif /*PYTORCH_H*/
