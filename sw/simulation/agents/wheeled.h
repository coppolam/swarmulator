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
  Wheeled(int i, const vector<float> &state, float tstep);
  void state_update();
  void animation();
};

#endif /*WHEELED_H*/