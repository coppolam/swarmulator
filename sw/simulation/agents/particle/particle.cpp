#include "particle.h"
#include "trigonometry.h"
#include "draw.h"

particle::particle(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[6];
  controller->set_saturation(1000.0);
}

std::vector<float> particle::state_update(std::vector<float> state)
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float v_x = 0.0;
  float v_y = 0.0;
  controller->get_velocity_command(ID, v_x, v_y);
  controller->saturate(v_x);
  controller->saturate(v_y);
  terminalinfo::debug_msg("V_x: "+std::to_string(v_x));
  terminalinfo::debug_msg("V_y: "+std::to_string(v_y));
  // moving = controller->moving;
  // happy = controller->happy;

  float vxr, vyr;
  rotate_xy(v_x, v_y, orientation, vxr, vyr);

  // Acceleration
  // p-controller
  state.at(4) = 2 * (vxr - state[2]); // Acceleration x
  state.at(5) = 2 * (vyr - state[3]); // Acceleration y

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x
  state.at(3) += state[5] * dt; // Velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

  return state;
};

void particle::animation()
{
  draw d;
  d.circle(param->scale());
}
