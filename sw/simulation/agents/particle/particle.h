#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

/**
 * This child class of agent implements the dynamics of simple accelerated particles using a kinematic model
 */
class particle: public Agent
{
public:
  /**
   * Constructor
   */
  particle(int i, vector<float> state, float tstep);

  /**
   * State update implementation
   */
  vector<float> state_update(vector<float> state);

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*PARTICLE_H*/