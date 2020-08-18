#ifndef GAS_AGENT_H
#define GAS_AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

class gas_agent: public Agent
{
public:
	gas_agent(int i, std::vector<float> state, float tstep);
	std::vector<float> state_update(std::vector<float> state);
	void animation();
};

#endif /*GAS_AGENT_H*/
