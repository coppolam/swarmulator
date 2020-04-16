#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "terminalinfo.h"
#include "randomgenerator.h"

using namespace std;

// Include behavior tree library
#include "bt/btFile.h"
using namespace BT;

class behavior_tree: public Controller
{
public:
  float v_x_ref, v_y_ref, ang;
  uint moving_timer; // Timer measuring how long a robot has been moving
  composite *tree; // Tree structure.
  blackboard BLKB; // Blackboard structure that ticks the tree during runtime.

  /** Initialize
   * Initialize the behavior tree
   */
  behavior_tree();

  /**
   * Function for lattice to all robots
   *
   * @param closest vector of ID of all closest neighbors
   * @param v_x
   * @param v_y
   */
  void lattice_all(const int &ID, const vector<uint> &closest, float &v_x, float &v_y);

  /**
   * Update the inputs and run the behavior tree command
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*BEHAVIOR_TREE_H*/
