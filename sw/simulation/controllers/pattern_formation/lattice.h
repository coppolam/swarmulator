#ifndef CONTROLLER_LATTICE_H
#define CONTROLLER_LATTICE_H

#include "lattice_basic.h"
#include "template_calculator.h"

class lattice : public lattice_basic
{
public:
  lattice();
  ~lattice() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*CONTROLLER_LATTICE_H*/