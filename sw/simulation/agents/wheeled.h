#ifndef WHEELED_H
#define WHEELED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

/**
 * This child class of agent implements the dynamics of simple accelerated wheeled vehicles using a kinematic model
 */
class Wheeled: public Agent
{
public:
  /**
   * Constructor
   */
  Wheeled(int i, vector<float> state, float tstep);

  /**
   * State update implementation
   */
  void state_update();

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*Wheeled_H*/