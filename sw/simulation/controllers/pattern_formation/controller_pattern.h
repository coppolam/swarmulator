#ifndef CONTROLLER_PATTERN
#define CONTROLLER_PATTERN

#include "controller_lattice_basic.h"

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "template_calculator.h"

class controller_pattern : public controller_lattice_basic
{
  uint moving_timer;
  int selected_action;

public:
  controller_pattern();
  ~controller_pattern() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_PATTERN*/