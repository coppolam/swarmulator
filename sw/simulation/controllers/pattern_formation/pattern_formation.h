#ifndef CONTROLLER_PATTERN
#define CONTROLLER_PATTERN

#include "lattice_basic.h"

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "template_calculator.h"

/**
 * This controller makes a pattern from a lattice.
 * It is a child to lattice_basic, which handles the lattice control
 */
class pattern_formation : public lattice_basic
{
  uint moving_timer; // Internal timer
  int selected_action; // Selected direction of motion within lattice

public:
  /**
   * @brief Construct a new pattern formation object
   */
  pattern_formation();

  /**
   * @brief Destroy the pattern formation object
   */
  ~pattern_formation() {};

  /**
   * @brief Get the velocity command object
   *
   * @param ID
   * @param v_x
   * @param v_y
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);

  /**
   * Draw the controller
   */
  virtual void animation(const uint16_t ID);
};

#endif /*CONTROLLER_PATTERN*/