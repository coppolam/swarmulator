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

using namespace std;

/*
* Parent class defining an agent. The dynamic implementation is handled in children classes.
*/
class Agent
{
public:
  // The constructor of agent is not defined here, but in the child class.
  // This makes it so that the agent to be used can be selected in settings.h
  virtual ~Agent(){}; // Destructor

  float dt;
  uint8_t ID;
  vector<float> state;
  bool moving;
  bool happy;
  float orientation;

  CONTROLLER controller; // Controller used by the agent. Defined in settings.h.

  /*
   * Returns the position of the agent along a certain dimension (i.e., North(0) and East(1))
   * This is used by the OmniscientObserver class in order to simulate the presence of sensors.
   */
  float get_position(uint8_t dim);

  /*
   * The agent class is only the parent class of a child class that specifies the dynamics and control
   * of a given agent (robot/animal/whatever). The state_update function is handled by the child class.
   */
  virtual void state_update() = 0;
};

#endif /*AGENT_H*/
