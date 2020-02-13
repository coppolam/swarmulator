#ifndef GREATER_THAN_H
#define GREATER_THAN_H

#include "condition.h"

namespace BT
{
/**
 * Class to check the great_than condition
 */
class greater_than : public condition
{
public:
  /**
   * @brief Constructor 1 for a new greater_than object
   */
  greater_than(std::string vehicle_name, size_t param = MAX_SIZE)
    : condition(vehicle_name, "greater_than", param)
  {
  }
  /**
   * @brief Constructor 2 for a new greater_than object
   */
  greater_than(std::string vehicle_name, size_t param, double value)
    : condition(vehicle_name, "greater_than", param, value)
  {
  }

  BT_Status update(blackboard *BLKB)
  {
    double data = BLKB->get(var.data());
    if (data > limit) { // Check whether the data in the condition is greater than than the limit
      return BH_SUCCESS;
    } else {
      return BH_FAILURE;
    }
  }
};

}

#endif // GREATER_THAN_H