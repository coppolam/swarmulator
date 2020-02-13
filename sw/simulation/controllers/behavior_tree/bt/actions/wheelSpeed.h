#ifndef WHEELSPEED_H
#define WHEELSPEED_H

#include "node.h"

namespace BT
{

struct wheelSpeed : public node {

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
   * @param left
   * @param right
   */
  wheelSpeed(double left, double right)
    : node("robot", "action", "wheelSpeed"),
      leftWheelSpeed(left),
      rightWheelSpeed(right)
  {
    // wheel speed randomly generated on interval [-0.5,0.5] with increments of 0.1
    vars.push_back(left);
    vars_upper_lim.push_back(0.5);
    vars_lower_lim.push_back(-0.5);

    vars.push_back(right);
    vars_upper_lim.push_back(0.5);
    vars_lower_lim.push_back(-0.5);
  }

  /**
   * @brief Construct a new random wheelSpeed object
   *
   */
  wheelSpeed()
    : wheelSpeed((rand() % 101) / 100. - 0.5, (rand() % 101) / 100. - 0.5)
  {}

private:
  double leftWheelSpeed;
  double rightWheelSpeed;
};

}

#endif // WHEELSPEED_H