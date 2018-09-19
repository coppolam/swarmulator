#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

class Particle: public Agent
{
public:
  Particle(int i, vector<float> state, float tstep);
  void state_update();
};

#endif /*PARTICLE_H*/