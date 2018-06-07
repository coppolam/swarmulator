#ifndef CONTROLLER_LATTICE_H
#define CONTROLLER_LATTICE_H

#include "controller_lattice_basic.h"
#include "template_calculator.h"

using namespace std;

class Controller_Lattice : public Controller_Lattice_Basic
{
  public: 
  Controller_Lattice();
  ~Controller_Lattice(){};
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_LATTICE_H*/