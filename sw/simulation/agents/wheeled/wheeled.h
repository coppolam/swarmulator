#ifndef WHEELED_H
#define WHEELED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "agent.h"

class wheeled: public Agent
{
  float r = 1; // Wheel radius
  float L = 10; // Distance between wheels
public:
  /**
   * @brief Construct a new wheeled object
   *
   * @param i ID
   * @param state Initial state
   * @param tstep Simulation time step
   */
  wheeled(int i, std::vector<float> state, float tstep);

  /**
   * State update over one time step
   *
   * @param state current state
   * @return vector<float> next state
   */
  std::vector<float> state_update(std::vector<float> state);

  /**
   * Drawing to animate the wheeled agent
   *
   */
  void animation();
};

#endif /*WHEELED_H*/
