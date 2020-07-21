#ifndef BEHAVIOR_TREE_FORAGE_H
#define BEHAVIOR_TREE_FORAGE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "terminalinfo.h"
#include "randomgenerator.h"
#include "template_calculator.h"
#define COMMAND_LOCAL 1  // Local frame

// Include behavior tree library
#include "bt/btFile.h"
using namespace BT;

class behavior_tree_forage: public Controller
{
public:
  composite *tree; // Tree structure.
  blackboard BLKB; // Blackboard structure that ticks the tree during runtime.

  std::vector<float> motion_p; // Probability of motion
  uint timer; // Timer measuring how long a robot has been moving
  bool explore;
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
  bool holds_food, choose;
  float state;

  /** Initialize
   * Initialize the behavior tree
   */
  behavior_tree_forage();

  /**
   * Update the inputs and run the behavior tree command
   */
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*BEHAVIOR_TREE_FORAGE_H*/
