#ifndef DECISION_H
#define DECISION_H

#include "node.h"

namespace BT
{

struct decision : public node {

  /**
   * @brief Update the value of the output on the blackboard
   *
   * @param BLKB
   * @return BT_Status
   */
  BT_Status update(blackboard *BLKB);

public:

  /**
   * @brief Construct a new wheelSpeed object
   *
   * @param left Left wheel speed (will be bounded within constructor)
   */
  decision(double probability)
    : node("robot", "action", "decision"),
      decision_probability(probability)
  {
    vars.push_back(probability);
    vars_lower_lim.push_back(0); // Lower limit
    vars_upper_lim.push_back(1.0); // Upper limit
  }

  /**
   * @brief Construct a new random wheelSpeed object
   * wheel speed randomly generated on interval [-0.5,0.5] with increments of 0.1
   */
  decision()
    : decision((rand() % 101) / 100. - 0)
  {}

private:
  double decision_probability;
};

}

#endif // DECISION_H
