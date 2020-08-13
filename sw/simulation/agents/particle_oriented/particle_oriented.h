#ifndef PARTICLE_ORIENTED_H
#define PARTICLE_ORIENTED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

#define COMMAND_LOCAL 1
/**
 * This child class of agent implements the dynamics of simple accelerated oriented vehicles using a kinematic model
 */
class particle_oriented: public Agent
{
public:
  /**
   * Constructor
   */
  particle_oriented(int i, std::vector<float> state, float tstep);

  /**
   * State update implementation
   */
  std::vector<float> state_update(std::vector<float> state);

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*PARTICLE_ORIENTED_H*/