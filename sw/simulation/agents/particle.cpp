#include "particle.h"

Particle::Particle(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
}

void Particle::update_position()
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float v_x, v_y;
  controller.get_velocity_command(ID, v_x, v_y);
  moving = controller.moving;
  keepbounded(v_x, -1.0, 1.0);
  keepbounded(v_y, -1.0, 1.0);

  // Acceleration
  state.at(4) = -15 * (state[2] - v_x); // Acceleration x
  state.at(5) = -15 * (state[3] - v_y);  // Acceleration y

  // Velocity
  state.at(2) = state[4] * dt; // velocity x
  state.at(3) = state[5] * dt; // velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // position y

};