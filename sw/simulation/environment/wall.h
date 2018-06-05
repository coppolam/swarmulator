#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

class Wall {
public:
  float dt; // timestep
  Wall(int i, const vector<float> &state, float tstep): Agent(i, state) {
    dt = tstep;
  }; // Make two versions for random initialization

  virtual void update_position();
};

#endif /*PARTICLE_H*/