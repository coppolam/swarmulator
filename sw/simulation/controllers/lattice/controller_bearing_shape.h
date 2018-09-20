#ifndef CONTROLLER_BEARING_SHAPE_H
#define CONTROLLER_BEARING_SHAPE_H

#include "controller_lattice_basic.h"

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "template_calculator.h"

using namespace std;

class Controller_Bearing_Shape : public Controller_Lattice_Basic
{
  uint moving_timer;
  int selected_action;

public:
  Controller_Bearing_Shape();
  ~Controller_Bearing_Shape() {};
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_BEARING_SHAPE_H*/