#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "settings.h"
#include "terminalinfo.h"

// Include all controllers here, as any of them may be loaded
#include "controller_cartesian.h"
#include "controller_bearing_shape.h"
#include "controller_lattice.h"
#include "controller_aggregate.h"
#include "controller_keep_aggregate.h"

using namespace std;

/*
* Parent class defining an agent. The dynamic implementation is handled in children classes.
*/
class Agent
{
public:
  virtual ~Agent() {};   // Destructor

  CONTROLLER controller; // Controller used by the agent. Defined in settings.h.

  float dt;
  uint8_t ID;
  vector<float> state;
  bool moving;
  bool happy;
  float orientation;

  vector<float> get_states();
  float get_position(uint8_t dim);
  uint8_t get_ID();
  
  virtual void state_update() = 0;
};

#endif /*AGENT_H*/
