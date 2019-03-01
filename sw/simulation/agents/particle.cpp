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

  float vx_global, vy_global;
  rotate_xy(v_x, v_y, state[6], vx_global, vy_global);
  state[6] = wrapToPi_f(state[6] + 0); // Orientation
  
  // Acceleration control
  float k_a = 15;
  state.at(4) = k_a * (vx_global - (state[4] * dt)); // Acceleration x global frame
  state.at(5) = k_a * (vy_global - (state[5] * dt)); // Acceleration y global frame

  // Velocity
  state.at(2) = v_x; // Velocity x local frame
  state.at(3) = v_y; // Velocity y local fram

  // Position
  state.at(0) += (state[4] * dt) * dt + 0.5 * state[4] * pow(dt, 2); // Position x global frame
  state.at(1) += (state[5] * dt) * dt + 0.5 * state[5] * pow(dt, 2); // Position y global frame

};

void Particle::animation()
{
  draw d;
  
  d.draw_triangle(param->scale());
  d.draw_circle_loop(param->scale());
}