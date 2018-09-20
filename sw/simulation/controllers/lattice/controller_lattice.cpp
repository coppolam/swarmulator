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


// if (o->request_distance(ID, closest[0]) > d_safe)
// {
//   uint count = 1;
//   for (size_t i = 0; i < state_ID.size(); i++)
//   {
//     v_b = wrapToPi_f(o->request_bearing(ID, state_ID[i]));
//     b_eq = t.get_preferred_bearing(beta_des, v_b);
//     v_r = get_attraction_velocity(o->request_distance(ID, state_ID[i]), b_eq);
//     latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
//     count++;
//   }
//   v_x = v_x / (float)count;
//   v_y = v_y / (float)count;
// }
// else
// {
//   v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
//   b_eq = t.get_preferred_bearing(beta_des, v_b);
//   v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);
//   latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
// }
//   bool canImove = check_motion(state_ID);

//   if (!canImove)
//   {
//     selected_action = -2;   // Reset actions
//     moving_timer = twait_1; // Reset moving timer
//   }
//   moving = false;
//   if (canImove)
//   {
//     if (selected_action > -1 && moving_timer < timelim && o->request_distance(ID, closest[0]) < 1.2)
//     {
//       actionmotion(selected_action, v_x, v_y);
//       moving = true;
//     }
//     else if (o->request_distance(ID, closest[0]) > d_safe)
//     {
//       uint count = 1;
//       for (size_t i = 0; i < state_ID.size(); i++)
//       {
//         v_b = wrapToPi_f(o->request_bearing(ID, state_ID[i]));
//         b_eq = t.get_preferred_bearing(beta_des, v_b);
//         v_r = get_attraction_velocity(o->request_distance(ID, state_ID[i]), b_eq);
//         latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
//         count++;
//       }
//       v_x = v_x / (float)count;
//       v_y = v_y / (float)count;
//     }
//     else
//     {
//       v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
//       b_eq = t.get_preferred_bearing(beta_des, v_b);
//       v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);
//       latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
//     }

//     if (moving_timer > twait_2)
//     {
//       moving_timer = 1;
//     }
//     else
//     {
//       moving_timer++;
//     }
// }

}