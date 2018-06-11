#include "particle.h"

Particle::Particle(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
  controller.set_saturation(1.0);
}

void Particle::update_position()
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float v_x, v_y;
  controller.get_velocity_command(ID, v_x, v_y);
  controller.saturate(v_x);
  controller.saturate(v_y);
  moving = controller.moving;

  // Acceleration
  state.at(4) = 15 * (v_x - state[2]); // Acceleration x
  state.at(5) = 15 * (v_y - state[3]);  // Acceleration y

  // Velocity
  state.at(2) = state[4] * dt; // Velocity x
  state.at(3) = state[5] * dt; // Velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

};