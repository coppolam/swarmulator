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

class pattern_formation : public lattice_basic
{
  uint moving_timer;
  int selected_action;

public:
  pattern_formation();
  ~pattern_formation() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*CONTROLLER_PATTERN*/