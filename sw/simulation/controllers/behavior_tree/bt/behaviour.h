#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <vector>
#include <string>

#include "blackboard.h"

namespace BT
{

// task status types
// Important that order is maintained with more important states as higher values
enum BT_Status {
  BH_INVALID,
  BH_SUCCESS,
  BH_RUNNING,
  BH_SUSPENDED,
  BH_FAILURE
};

class node
/**
 * base class for actions, conditions and composites
 */
{
public:
  node(std::string Name, std::string Type, std::string Function)
    : m_eStatus(BH_INVALID),
      name(Name),
      type(Type),
      function(Function),
      tick_counter(0)
  {}

  virtual ~node()
  {
  }

  // general
  BT_Status tick(blackboard *BLKB);
  virtual BT_Status update(blackboard *BLKB) = 0;
  virtual void onInitialise() {}
  virtual void onTerminate(BT_Status) {}

  BT_Status m_eStatus;
  std::string name;
  std::string type;
  std::string function;
  size_t tick_counter;
  std::vector<double> vars;      // node internal variables
  std::vector<double> vars_upper_lim;
  std::vector<double> vars_lower_lim;
};

}
#endif // BEHAVIOUR_H
