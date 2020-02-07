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

using namespace std;

class Controller_Pattern : public Controller_Lattice_Basic
{
  uint moving_timer;
  int selected_action;

public:
  Controller_Pattern();
  ~Controller_Pattern() {};
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_PATTERN*/