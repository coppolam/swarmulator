#include "bug.h"
#include "draw.h"

using namespace std;

bug::bug(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = 0.0;
}

vector<float> bug::state_update(vector<float> state)
{ 
  /*** Include your model here ***/ 
  float psi, v_x, vx_global,vy_global,dpsirate;
  controller->get_velocity_command(ID, psi,v_x); // here psi is the desired heading, while v_x is desired velocity in x in body frame
  
  //state: 0: y, 1:x, 2:dy/dt,3:dx/dt, 4:a_y, 5:a_x, 6: psi, 7:psi_rate
  float k_psi = 4.0;
  dpsirate = k_psi*(psi-state.at(6));

  state.at(7) = dpsirate;
  state.at(6) += state[7] * dt;
  terminalinfo::debug_msg(std::to_string(state.at(6)));
  
  rotate_xy(v_x, 0, state[6], vx_global, vy_global); // compute desired velocities in global frame


  float ka = 2.0;
  state.at(4) = ka * (vx_global - state[2]); // Acceleration global frame
  state.at(5) = ka * (vy_global - state[3]); // Acceleration global frame

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x global frame
  state.at(3) += state[5] * dt; // Velocity y global frame

  // Position -- tailor expansion
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

  return state;
  
}

void bug::animation()
{
  draw d;
  /*** Draw your agent here. ***/
  d.circle(param->scale());
}
