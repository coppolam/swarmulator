#include "wheeled.h"
#include "draw.h"

wheeled::wheeled(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
}

void wheeled::state_update()
{ 
  float leftwheelspeed, rightwheelspeed;
  controller.get_velocity_command(ID, leftwheelspeed, rightwheelspeed);
  
  cout << leftwheelspeed << " " << rightwheelspeed << endl;
  controller.saturate(leftwheelspeed);
  controller.saturate(rightwheelspeed);
  moving = controller.moving;
  happy = controller.happy;

  float r = 1; // Wheel radius
  float L = 10; // Distance between wheels

  float v_x = r/2 * leftwheelspeed + r/2 * rightwheelspeed;
  float v_y = 0.0;
  float psi_rate = - r/L * leftwheelspeed + r/L * rightwheelspeed;
  cout << v_x << " " << rightwheelspeed << endl;

  float vxr, vyr;
  rotate_xy(v_x, v_y, state[6], vxr, vyr); // Local frame to global frame

  // Orientation
  state.at(7) = psi_rate; // Orientation rate
  state.at(6) += state.at(7); // Orientation

  // Velocity
  state.at(2) = vxr; // Velocity x
  state.at(3) = vyr; // Velocity y

  // Position
  state.at(0) += state[2] * dt; // Position x
  state.at(1) += state[3] * dt; // Position y
}


void wheeled::animation()
{
  draw d;
  d.draw_triangle(param->scale());
  d.draw_circle_loop(rangesensor);
}
