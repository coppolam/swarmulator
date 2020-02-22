#ifndef BEHAVIOR_TREE_WHEELED_H
#define BEHAVIOR_TREE_WHEELED_H

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

class behavior_tree_wheeled: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;
  terminalinfo ti;

public:
  bool moving; // Robot sets it if it is moving.
  float v_x_ref, v_y_ref, ang;
  uint walltimer; // Timer to set the time for a wall avoidance maneuver
  uint moving_timer; // Timer measuring how long a robot has been moving
  composite *tree; // Tree structure.
  blackboard BLKB; // Blackboard structure that ticks the tree during runtime.
  random_generator rg; // Random generator

  /** Initialize
   * Initialize the behavior tree
   */
  behavior_tree_wheeled();

  /** Attraction
   * Attraction function at distance u
   * @param u distance
   * @return (float) attraction component
   */
  float f_attraction(float u);

  /**
   * Function to get the total attraction/repulsion velocity
   * @param u distance
   * @return (float) attraction+repulsion velocity component
   */
  float get_attraction_velocity(float u);

  /**
   * Function to give the commands for robots to stay in a lattice
   * @param ID ID of robot in question
   * @param state_ID List of nearby robots (within sensor range)
   * @param v_x v_x component of attraction/repulsion
   * @param v_y v_y component of attraction/repulsion
   */
  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);

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

#endif /*BEHAVIOR_TREE_WHEELED_H*/
