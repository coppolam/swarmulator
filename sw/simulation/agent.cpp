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