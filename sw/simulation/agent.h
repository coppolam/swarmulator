#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "settings.h"
#include "terminalinfo.h"

// Include all controllers here
#include "controller_cartesian.h"
#include "controller_bearing_shape.h"
#include "controller_lattice.h"
#include "controller_aggregate.h"
#include "controller_keep_aggregate.h"

using namespace std;

class Agent
{
public:
  virtual ~Agent(){};

  CONTROLLER controller;

  float dt;
  uint8_t ID;
  vector<float> state;
  bool moving;
  float orientation;
  
  vector<float> get_states();
  float get_position(uint8_t dim);
  uint8_t get_ID();

  virtual void update_position() = 0; // Defined by lower level functions
};

#endif /*AGENT_H*/
