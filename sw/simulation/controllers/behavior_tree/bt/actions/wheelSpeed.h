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
   * @param left Left wheel speed (will be bounded within constructor)
   * @param right Right wheel speed (will be bounded within constructor)
   */
  wheelSpeed(double left, double right)
    : node("robot", "action", "wheelSpeed"),
      leftWheelSpeed(left),
      rightWheelSpeed(right)
  {
    vars.push_back(left);
    vars_upper_lim.push_back(0); // Lower limit
    vars_lower_lim.push_back(1.0); // Upper limit

    vars.push_back(right);
    vars_upper_lim.push_back(0); // Lower limit
    vars_lower_lim.push_back(1.0); // Upper limit
  }

  /**
   * @brief Construct a new random wheelSpeed object
   * wheel speed randomly generated on interval [-0.5,0.5] with increments of 0.1
   */
  wheelSpeed()
    : wheelSpeed((rand() % 101) / 100. - 0, (rand() % 101) / 100. - 0)
  {}

private:
  double leftWheelSpeed;
  double rightWheelSpeed;
};

}

#endif // WHEELSPEED_H