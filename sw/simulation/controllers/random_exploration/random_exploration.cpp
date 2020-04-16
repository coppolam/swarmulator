#include "random_exploration.h"
#include "draw.h"
#include <cmath>

random_exploration::random_exploration() : Controller()
{
  moving = false;
}

void random_exploration::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0.0;
  v_y = 0.0;

  get_lattice_motion_all(ID, v_x, v_y);

  float v_x_ref = v_x_old;
  float v_y_ref = v_y_old;

  float ang = rg.uniform_float(-M_PI, M_PI);
  if (!moving) {
    polar2cart(0.5, ang, v_x_ref, v_y_ref);
    moving = true;
  }

  v_x_old = v_x_ref;
  v_y_old = v_y_ref;
  v_x += v_x_ref;
  v_y += v_y_ref;
  wall_avoidance(ID, v_x, v_y);

}