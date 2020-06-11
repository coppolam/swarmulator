#ifndef PYTORCH_H
#define PYTORCH_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

class pytorch: public Controller
{
public:
  pytorch(): Controller() {};
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
};

#endif /*PYTORCH_H*/
