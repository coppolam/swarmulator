#include "random_exploration.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>

random_exploration::random_exploration() : Controller() {}

void random_exploration::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0.5;
  float v_y_ref = v_y_old;
  float ang = rg.uniform_float(-M_PI, M_PI);
  v_y_old = v_y_ref;
  v_y += ang;
  wall_avoidance_turn(ID, v_x, v_y);
}

void random_exploration::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}