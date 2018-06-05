#include "controller_lattice.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

Controller_Lattice::Controller_Lattice() : Controller_Lattice_Basic()
{
  bdes.push_back(deg2rad(0));
  // bdes.push_back(deg2rad(  45));
  bdes.push_back(deg2rad(90));
  // bdes.push_back(deg2rad(  135));
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

  // What commands does this give?
  float v_b  = wrapToPi_f(o->request_bearing(ID, closest[0]));
  float b_eq = t.get_preferred_bearing(bdes, v_b);
  float v_r  = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);

  latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
}