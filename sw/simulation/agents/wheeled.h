#ifndef WHEELED_H
#define WHEELED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

using namespace std;

class Wheeled : public Agent
{
  public:
    float dt; // timestep
    Wheeled(int i, const vector<float> &state, float tstep) : Agent(i, state)
    {
        dt = tstep;
    }; // Make two versions for random initialization

    virtual void update_position();
};

#endif /*WHEELED_H*/