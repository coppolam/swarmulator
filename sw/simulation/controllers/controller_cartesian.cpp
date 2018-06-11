#include "controller_cartesian.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"

// Only one of the following can work
// #define FORCED    // Forces to use a specific adjacency matrix as specified in "adjacencymatrix.txt"
#define KNEAREST // Use a k-nearest topology according to the second argument
// #define ROGUE     // Would you like an agent to go rogue?
// #define rogueID 0 // ID of agent that goes rogue.

float Controller_Cartesian::f_attraction(float u)
{
  // float w = log((_ddes/_kr-1)/exp(-_ka*_ddes))/_ka;
  float w = 1.5;
  return  1 / (1 + exp(-_ka * (u - w))) + 1 / (1 + exp(-_ka * (u + w))) - 1; //% sigmoid function -- long-range
}

float Controller_Cartesian::get_attraction_velocity(float u)
{
  return f_attraction(u) + f_repulsion(u);
}

void Controller_Cartesian::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

#ifdef KNEAREST

  vector<int> closest = o->request_closest(ID);
  for (uint8_t i = 0; i < knearest; i++) {
    v_x += get_attraction_velocity(o->request_distance_dim(ID, closest[i], 0));
    v_y += get_attraction_velocity(o->request_distance_dim(ID, closest[i], 1));
  }

#endif

#ifdef FORCED

  uint8_t i = 0;
  std::ifstream infile("adjacencymatrix.txt");
  bool mat[nagents * nagents];
  while (i < (nagents * nagents)) {
    infile >> mat[i];
    i++;
  }

  for (i = 0; i < nagents; i++) {
    if (i != ID) {
      v_x += (get_attraction_velocity(o->request_distance(ID, i, 0)) * mat[ID * nagents + i]);
      v_y += (get_attraction_velocity(o->request_distance(ID, i, 0)) * mat[ID * nagents + i]);
    }
  }
#endif

#ifdef ROGUE
  if (ID == rogueID && (simulation_realtimefactor * simulation_time / 1000000.0) > 5.0) {
    v += 0.5;
  }
#endif
};