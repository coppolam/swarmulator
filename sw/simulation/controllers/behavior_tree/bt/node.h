#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <vector>
#include <string>

#include "blackboard.h"

namespace BT
{

/**
 * @brief Task status types
 * Important that order is maintained with more important states as higher values
 *
 */
enum BT_Status {
  BH_INVALID,
  BH_SUCCESS,
  BH_RUNNING,
  BH_SUSPENDED,
  BH_FAILURE
};

/**
 * Base class for actions, conditions and composites
 */
class node
{
public:
  /**
   * @brief Construct a new node object
   *
   * @param Name
   * @param Type
   * @param Function
   */
  node(std::string Name, std::string Type, std::string Function)
    : m_eStatus(BH_INVALID),
      name(Name),
      type(Type),
      function(Function),
      tick_counter(0)
  {}

  /**
   * @brief Destroy the node object
   *
   */
  virtual ~node()
  {
  }

  // General

  /**
   * @brief Tick the current behavior tree
   *
   * @param BLKB Blackboard object keeping track of the current state
   * @return BT_Status
   */
  BT_Status tick(blackboard *BLKB);

  /**
   * @brief Update the status of the behavior tree
   *
   * @param BLKB Blackboard object keeping track of the current state
   * @return BT_Status
   */
  virtual BT_Status update(blackboard *BLKB) = 0;

  /**
   * @brief
   *
   */
  virtual void onInitialise() {}

  /**
   * @brief
   *
   */
  virtual void onTerminate(BT_Status) {}

  BT_Status m_eStatus;
  std::string name;
  std::string type;
  std::string function;
  size_t tick_counter;
  std::vector<double> vars; // Node internal variables
  std::vector<double> vars_upper_lim;
  std::vector<double> vars_lower_lim;
};

}
#endif // BEHAVIOUR_H
