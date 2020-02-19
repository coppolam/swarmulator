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
public:
	wheeled(int i, vector<float> state, float tstep);
	void state_update();
	void animation();
};

#endif /*WHEELED_H*/
