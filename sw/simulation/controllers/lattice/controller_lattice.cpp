#include "controller_lattice.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "trigonometry.h"

Controller_Lattice::Controller_Lattice() : Controller_Lattice_Basic()
{
  beta_des.push_back(deg2rad(0));
  // beta_des.push_back(deg2rad(  45));
  beta_des.push_back(deg2rad(90));
  // beta_des.push_back(deg2rad(  135));
};

void Controller_Lattice::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  vector<bool> state(8, 0);
  vector<int> state_ID;
  // The ID is just used for simulation purposes
  t.assess_situation(ID, state, state_ID);

  v_x = 0;
  v_y = 0;

  // Which neighbors can you sense within the range?
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

}