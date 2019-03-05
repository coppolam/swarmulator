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
  manual = false;
}

void Particle::state_update()
{
  // NED frame
  // x+ towards North
  // y+ towards East

  float vx_des, vy_des;
  float vx_global, vy_global;
  if (!manual) {
    controller.get_velocity_command(ID, vx_des, vy_des); // Command comes out in the local frame
    controller.saturate(vx_des);
    controller.saturate(vy_des);
    rotate_xy(vx_des, vy_des, state[6], vx_global, vy_global);
    state[6] = wrapToPi_f(state[6]); // Orientation
    // Acceleration control
    float k_a = 15;
    state.at(4) = k_a * (vx_global - (state[4] * dt)); // Acceleration x global frame
    state.at(5) = k_a * (vy_global - (state[5] * dt)); // Acceleration y global frame

  }
  else {
    state.at(4) = manualx;
    state.at(5) = manualy;
  }
  moving = controller.moving;
  happy = controller.happy;

  // Velocity
  float vx,vy;
  rotate_xy(state[4] * dt, state[5] * dt, -state[6], vx, vy);
  state.at(2) = vx; // Velocity x local frame
  state.at(3) = vy; // Velocity y local frame
  
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