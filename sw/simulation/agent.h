#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include "../../conf/parameters.h"

#include CONTROLLER_INCLUDE

#include "terminalinfo.h"
using namespace std;

class Agent
{
  vector<float> inputs;
  vector<float> actions;

public:
  Agent(uint8_t i, const vector<float> &s); // Make two versions for random initialization
  ~Agent();

  uint8_t ID;
  vector<float> outputs;
  vector<float> state;

  CONTROLLER controller;

  vector<float> get_states();
  void select_action();
  virtual void update_position() = 0;
  float get_position(uint8_t dim);
  uint8_t get_ID();
};

#endif /*AGENT_H*/
