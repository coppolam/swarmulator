#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "settings.h"
#include "terminalinfo.h"

#include "controller_cartesian.h"
#include "controller_bearing_shape.h"
#include "controller_lattice.h"
#include "controller_aggregate.h"
#include "controller_keep_aggregate.h"

using namespace std;

class Agent
{
  vector<float> inputs;
  vector<float> actions;

public:
  virtual ~Agent(){};

  float dt;
  uint8_t ID;
  vector<float> outputs;
  vector<float> state;
  bool moving;

  CONTROLLER controller;

  virtual void update_position() = 0;
  vector<float> get_states();
  float get_position(uint8_t dim);
  uint8_t get_ID();
};

#endif /*AGENT_H*/
