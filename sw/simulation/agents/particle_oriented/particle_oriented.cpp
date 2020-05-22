#include "particle_oriented.h"
#include "trigonometry.h"
#include "randomgenerator.h"
#include "draw.h"

particle_oriented::particle_oriented(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[6];
  controller->set_saturation(0.5);
  manual = false;
}

vector<float> particle_oriented::state_update(vector<float> state)
{
  // NED frame
  // x+ towards North
  // y+ towards East
  float vx_des, vy_des = 0.;
  float vx_global, vy_global, dpsirate;
  // if (!manual) {
  controller->get_velocity_command(ID, vx_des, dpsirate); // Command comes out in the local frame
  // dpsi_rate = 0;
  // } else {
  //   vx_des = manualx;
  //   vy_des = manualy;
  //   dpsi_rate = manualpsi_delta;
  // }
  // controller->saturate(vx_des);
  // controller->saturate(vy_des);
#if COMMAND_LOCAL
  rotate_xy(vx_des, vy_des, state[6], vx_global, vy_global);
#else
  vx_global = vx_des;
  vy_global = vy_des;
#endif
  state.at(7) = dpsirate;
  state.at(6) += state[7] * dt;
  state.at(6) = wrapToPi_f(state[6]); // Orientation
  orientation = state.at(6);

  // Acceleration control
  float ka = 2;
  state.at(4) = ka * (vx_global - state[2]); // Acceleration global frame
  state.at(5) = ka * (vy_global - state[3]); // Acceleration global frame
  moving = controller->moving;
  happy = controller->happy;

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x global frame
  state.at(3) += state[5] * dt; // Velocity y global frame

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x global frame
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y global frame

  return state;
};

void particle_oriented::animation()
{
  draw d;

  d.triangle(param->scale());
  d.circle_loop(rangesensor);
}
