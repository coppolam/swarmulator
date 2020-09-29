#include "agent.h"
#include "randomgenerator.h"

Agent::Agent()
{
  activated = false;
}

float Agent::get_position(uint16_t dim)
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

float Agent::get_state(const uint16_t i)
{
  float noise = rg.gaussian_float(0.0, NOISE_R);
  return state[i] + noise;
}
