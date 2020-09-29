#ifndef BUG_H
#define BUG_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

class bug: public Agent
{
public:
	bug(int i, std::vector<float> state, float tstep);
	std::vector<float> state_update(std::vector<float> state);
	void animation();
};

#endif /*BUG_H*/
