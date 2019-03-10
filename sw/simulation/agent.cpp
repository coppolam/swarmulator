#include "agent.h"

float Agent::get_position(uint8_t dim)
{
  if (dim < 3) {
    return state[dim];
  }
  return 0;
}

float Agent::get_orientation()
{
  return state[6];
}

float Agent::get_state(uint8_t i)
{
  return state[i];
}