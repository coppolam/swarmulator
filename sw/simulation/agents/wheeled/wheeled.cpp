#include "wheeled.h"
#include "draw.h"

wheeled::wheeled(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[6];
}

vector<float> wheeled::state_update(vector<float> state)
{
  float leftwheelspeed, rightwheelspeed;

  controller->get_velocity_command(ID, leftwheelspeed, rightwheelspeed);
  controller->saturate(leftwheelspeed);
  controller->saturate(rightwheelspeed);
  moving = controller->moving;
  happy = controller->happy;

  // Model
  float v_x = r / 2 * leftwheelspeed + r / 2 * rightwheelspeed;
  float v_y = 0.0;
  float psi_rate = - r / L * leftwheelspeed + r / L * rightwheelspeed;

  // Orientation
  state.at(7) = psi_rate; // Orientation rate
  state.at(6) += state.at(7); // Orientation
  orientation = state.at(6);

  // Velocity
  float vxr, vyr;
  rotate_xy(v_x, v_y, orientation, vxr, vyr); // Local frame to global frame
  state.at(2) = vxr; // Velocity x
  state.at(3) = vyr; // Velocity y

  // Position
  state.at(0) += state[2] * dt; // Position x
  state.at(1) += state[3] * dt; // Position y

  return state;
}


void wheeled::animation()
{
  draw d;
  d.triangle(param->scale());
  d.circle_loop(rangesensor);
}
