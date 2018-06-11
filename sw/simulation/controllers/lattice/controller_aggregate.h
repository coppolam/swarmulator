#ifndef CONTROLLER_AGGREGATE_H
#define CONTROLLER_AGGREGATE_H

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "auxiliary.h"
#include "template_calculator.h"
#include "controller_lattice_basic.h"

using namespace std;

class Controller_Aggregate : public Controller_Lattice_Basic
{
  int moving_timer;
  int selected_action;
  int motion_dir = 0;   // Use 0 for random or 1-8 to specify a direction.

public:
  Controller_Aggregate();
  ~Controller_Aggregate() {};
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_AGGREGATE_H*/