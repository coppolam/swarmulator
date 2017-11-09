#include "agent.h"

Agent::Agent(uint8_t i, const vector<float> &s):
  situation(8, 0)
{
  this->state = s;
  ID = i;
};

Agent::~Agent() {};

std::vector<float> Agent::get_states()
{
  return state;
};

float Agent::get_position(uint8_t dim)
{
  if (dim < 3) {
    return state[dim];
  }

  return 0;
}

void Agent::select_action() {};

uint8_t Agent::get_ID()
{
  return ID;
};