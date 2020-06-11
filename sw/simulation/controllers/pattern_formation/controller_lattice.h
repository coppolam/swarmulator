#ifndef CONTROLLER_LATTICE_H
#define CONTROLLER_LATTICE_H

#include "controller_lattice_basic.h"
#include "template_calculator.h"

class controller_lattice : public controller_lattice_basic
{
public:
  controller_lattice();
  ~controller_lattice() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_LATTICE_H*/