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
  // Agent(uint8_t i, const vector<float> &s, float tstep);
  virtual ~Agent(){};

  float dt;
  uint8_t ID;
  vector<float> outputs;
  vector<float> state;
  bool moving;

  CONTROLLER controller;

  vector<float> get_states();
  virtual void update_position()=0;
  float get_position(uint8_t dim);
  uint8_t get_ID();
};

#endif /*AGENT_H*/
