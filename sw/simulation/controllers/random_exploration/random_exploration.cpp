#include "random_exploration.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>

random_exploration::random_exploration() : Controller()
{
}

void random_exploration::get_velocity_command(const uint16_t ID, float &v_x, float &psirate)
{
  v_x = 0.5;
  psirate = rg.uniform_float(-M_PI, M_PI);
  wall_avoidance_turn(ID, v_x, psirate);
}

void random_exploration::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}