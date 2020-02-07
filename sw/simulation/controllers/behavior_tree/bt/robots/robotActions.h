#ifndef ROBOTACTIONS_H
#define ROBOTACTIONS_H

#include <stdlib.h>
#include "../behaviour.h"
#include <cmath>

namespace BT
{

node *getAction(std::string action, std::vector<double> inputs);
node *getAction(size_t func = (size_t) - 1);

// wheel speed randomly generated on interval [-0.5,0.5] with increments of 0.1
struct wheelSpeed : public node {
  BT_Status update(blackboard *BLKB);

public:
  wheelSpeed(double left, double right)
    : node("robot", "action", "wheelSpeed"),
      leftWheelSpeed(left),
      rightWheelSpeed(right)
  {
    vars.push_back(leftWheelSpeed);
    vars_upper_lim.push_back(0.5);
    vars_lower_lim.push_back(-0.5);

    vars.push_back(rightWheelSpeed);
    vars_upper_lim.push_back(0.5);
    vars_lower_lim.push_back(-0.5);

    if (std::fabs(leftWheelSpeed) > 0.5 || std::fabs(rightWheelSpeed) > 0.5) {
      std::cout << "ACTION: " << leftWheelSpeed << " " << rightWheelSpeed << std::endl;
    }
  }

  wheelSpeed()
    : wheelSpeed((rand() % 101) / 100. - 0.5, (rand() % 101) / 100. - 0.5)
  {}

private:
  double leftWheelSpeed;
  double rightWheelSpeed;
};

}
#endif // ROBOTACTIONS_H
