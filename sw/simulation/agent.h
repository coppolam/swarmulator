#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "settings.h"
#include "terminalinfo.h"
#include "includes_controllers.h"

using namespace std;

/**
 * Parent class defining an agent. The dynamic implementation is handled in children classes.
 */
class Agent
{
public:
  /**
   * The constructor of agent is not defined here, but in the child class.
   * This makes it so that the agent to be used can be selected in settings.h
   */

  /**
   * Destructor
   */
  virtual ~Agent() {};

  uint8_t ID; // ID of agent
  float dt;
  vector<float> state; // State vector
  float orientation; // Orientation
  bool moving;
  bool happy;
  float manualx, manualy;
  bool manual;
  float manualpsi_delta;
  random_generator rg;
  CONTROLLER controller; // Controller used by the agent. Defined in settings.h.

  /**
   * Returns the position of the agent along a certain dimension (i.e., North(0) and East(1))
   * This is used by the OmniscientObserver class in order to simulate the presence of sensors.
   */
  float get_position(uint8_t dim);

  /**
   * @brief Get the orientation object
   *
   * @return float
   */
  float get_orientation();

  /**
   * @brief Get the state along a given dimension
   *
   * @param dim dimension
   * @return float
   */
  float get_state(uint8_t dim);

  /**
   * The agent class is only the parent class of a child class that specifies the dynamics and control
   * of a given agent (robot/animal/whatever). The state_update function is handled by the child class.
   */
  virtual vector<float> state_update(vector<float>) = 0;

  /**
   * The agent class is only the parent class of a child class that specifies the dynamics and control
   * of a given agent (robot/animal/whatever). The way that the agent looks is specified in the child class.
   */
  virtual void animation() = 0;
};

#endif /*AGENT_H*/
