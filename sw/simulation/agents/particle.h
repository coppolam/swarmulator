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
class Particle: public Agent
{
public:
  /**
   * Constructor
   */
  Particle(int i, vector<float> state, float tstep);

  /**
   * State update implementation
   */
  void state_update();

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*PARTICLE_H*/