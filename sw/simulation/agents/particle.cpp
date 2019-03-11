#include "particle.h"
#include "trigonometry.h"
#include "randomgenerator.h"
// extern "C" {
#include "ndi_follower.h"
// }
#include "draw.h"

Particle::Particle(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  random_generator rg;
  orientation = state[6];
  controller.set_saturation(0.5);
  manual = false;
}

void Particle::state_update()
{
  // NED frame
  // x+ towards North
  // y+ towards East
  float vx_des, vy_des;
  float vx_global, vy_global, dpsi_rate;
  if (!manual) {
    controller.get_velocity_command(ID, vx_des, vy_des); // Command comes out in the local frame
    dpsi_rate = 0;
  } else {
    vx_des = manualx;
    vy_des = manualy;
    dpsi_rate = manualpsi_delta;
  }
  controller.saturate(vx_des);
  controller.saturate(vy_des);
  rotate_xy(vx_des, vy_des, state[6], vx_global, vy_global);

  state.at(7) = dpsi_rate;
  state.at(6) += dpsi_rate * dt;
  state.at(6) = wrapToPi_f(state[6]); // Orientation

  // Acceleration control
  float ka = 1;
  state.at(4) = ka * (vx_global - state[2]); // Acceleration global frame
  state.at(5) = ka * (vy_global - state[3]); // Acceleration global frame
  moving = controller.moving;
  happy = controller.happy;

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x global frame
  state.at(3) += state[5] * dt; // Velocity y global frame

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x global frame
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y global frame
};

void Particle::animation()
{
  draw d;

  d.draw_triangle(param->scale());
  d.draw_circle_loop(param->scale());
}