#include "random_exploration.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>

#define SENSOR_MAX_RANGE 1.8

random_exploration::random_exploration() : Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
}

void random_exploration::get_velocity_command(const uint16_t ID, float &v_x, float &psirate)
{
  v_x = 0.5;
  psirate = rg.uniform_float(-M_PI, M_PI);
  wall_avoidance_turn(ID, v_x, psirate, SENSOR_MAX_RANGE);
}

void random_exploration::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(SENSOR_MAX_RANGE);
}