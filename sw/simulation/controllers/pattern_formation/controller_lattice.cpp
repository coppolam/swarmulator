#include "controller_lattice.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "trigonometry.h"

Controller_Lattice::Controller_Lattice() : Controller_Lattice_Basic()
{
  beta_des.push_back(0.0);
  beta_des.push_back(atan(_ddes_y / _ddes_x));
  beta_des.push_back(M_PI / 2.0);
  beta_des.push_back(M_PI / 2.0 + atan(_ddes_x / _ddes_y));
};

void Controller_Lattice::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<bool> state(8, 0);
  vector<int> state_ID;
  // The ID is just used for simulation purposes
  t.assess_situation(ID, state, state_ID);
  vector<int> closest = o.request_closest(ID);
  get_lattice_motion_all(ID, state_ID, closest, v_x, v_y);
}