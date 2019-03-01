#include "particle.h"
#include "trigonometry.h"
#include "randomgenerator.h"
#include "draw.h"

Particle::Particle(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  random_generator rg;
  orientation = state[6];
  controller.set_saturation(1.0);
}

void Particle::state_update()
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
  rotate(v_x, v_y, state[6], vxr, vyr);
  state[6] = wrapToPi_f(state[6] + 0); // Orientation

  // Acceleration
  state.at(4) = 15 * (vxr - state[2]); // Acceleration x
  state.at(5) = 15 * (vyr - state[3]); // Acceleration y

  // Velocity
  state.at(2) = state[4] * dt; // Velocity x
  state.at(3) = state[5] * dt; // Velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

};

void Particle::animation()
{
  draw d;
  
  d.draw_triangle(param->scale());
  d.draw_circle_loop(param->scale());
}