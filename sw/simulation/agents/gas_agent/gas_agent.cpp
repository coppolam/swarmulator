#include "gas_agent.h"
#include "draw.h"

using namespace std;

gas_agent::gas_agent(int i, vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  orientation = state[6];
}

vector<float> gas_agent::state_update(vector<float> state)
{ 
  float v_x, v_y, vx_global, vy_global, dpsirate;;
  controller->get_velocity_command(ID, v_x, dpsirate);
  
  rotate_xy(v_x, 0, state[6], vx_global, vy_global);
  state.at(7) = dpsirate;
  state.at(6) += state[7] * dt;
  orientation = wrapToPi_f(state[6]);

  // Acceleration control
  float ka = 2;
  state.at(4) = ka * (vx_global - state[2]); // Acceleration global frame
  state.at(5) = ka * (vy_global - state[3]); // Acceleration global frame

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x global frame
  state.at(3) += state[5] * dt; // Velocity y global frame
  // state.at(2) = vx_global;
  // state.at(3) = vy_global;

  // Position
  // state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x global frame
  // state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y global frame

  state.at(0) += vx_global * dt; // Position x global frame
  state.at(1) += vy_global * dt; // Position y global frame

  return state;
  /*** Include your model here ***/ 
}

void gas_agent::animation()
{
  draw d;
  /*** Draw your agent here. ***/
  d.triangle(param->scale());
}
