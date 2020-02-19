#include "particle.h"
#include "trigonometry.h"
#include "draw.h"

particle::particle(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
  controller.set_saturation(1.0);
}

void particle::state_update()
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float v_x, v_y;
  controller.get_velocity_command(ID, v_x, v_y);
  controller.saturate(v_x);
  controller.saturate(v_y);
  moving = controller.moving;
  happy = controller.happy;

  float vxr, vyr;
  rotate_xy(v_x, v_y, orientation, vxr, vyr);

  // Acceleration
  state.at(4) = 1 * (vxr - state[2]); // Acceleration x
  state.at(5) = 1 * (vyr - state[3]); // Acceleration y

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x
  state.at(3) += state[5] * dt; // Velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

};

void particle::animation()
{
  draw d;
  d.draw_circle(param->scale());
  d.draw_circle_loop(rangesensor);
}
