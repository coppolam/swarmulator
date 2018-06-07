#include "agent.h"

vector<float> Agent::get_states()
{
  return state;
};

float Agent::get_position(uint8_t dim)
{
  if (dim < 3)
  {
    return state[dim];
  }

  return 0;
}

uint8_t Agent::get_ID()
{
  return ID;
};