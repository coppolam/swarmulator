#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

// Include behavior tree library
#include "bt/btFile.h"
using namespace BT;

class behavior_tree: public Controller
{ 
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;

public:
  int walltimer; // Timer to set the time for a wall avoidance maneuver
  composite *tree; // Tree structure.
  blackboard BLKB; // Blackboard structure that ticks the tree during runtime.
  float v_x_ref, v_y_ref;

  /* Initialize
   * Initialize the behavior tree
   */
  behavior_tree();
   /* Attraction
    * Attraction function at distance u
    */
  float f_attraction(float u);

  /*
   * Function to get the total attraction/repulsion velocity
   */
  float get_attraction_velocity(float u);

  /*
   * Function to give the commands for robots to stay in a lattice
   */
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);

  /*
   * Update the inputs and run the behavior tree command
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*BEHAVIOR_TREE_H*/
