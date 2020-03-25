#ifndef WHEELED_H
#define WHEELED_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include "agent.h"

using namespace std;

class wheeled: public Agent
{
  float r = 1; // Wheel radius
  float L = 10; // Distance between wheels
public:
  wheeled(int i, vector<float> state, float tstep);
  vector<float> state_update(vector<float> state);
  void animation();
};

#endif /*WHEELED_H*/
