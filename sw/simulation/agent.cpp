#include "agent.h"

Agent::Agent(int i, const vector<float> &s):situation(8,0)
{
	info_msg ("Agent " + std::to_string(i) + " created at position (" + std::to_string(s[0]) + "," + std::to_string(s[1]) + ")");
	this->state = s;
	ID = i;
	controller.set_saturation(1.0);
};

Agent::~Agent(){};

std::vector<float> Agent::get_states()
{
	return state;
};

float Agent::get_position(int dim)
{
	if (dim < 3)
	{
		return state[dim];
	}

	return 0;
}

void Agent::select_action(){};

int Agent::get_ID()
{
	return ID;
};