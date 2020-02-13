#ifndef LESS_THAN_H
#define LESS_THAN_H

#include "condition.h"

namespace BT
{
/**
 * Class for the less_than condition
 *
 */
class less_than : public condition
{
public:
  /**
   * Constructor 1 for a new less_than object
   */
  less_than(std::string vehicle_name, size_t param = MAX_SIZE)
    : condition(vehicle_name, "less_than", param)
  {
  }
  /**
   * Constructor 2 for a new less_than object
   */
  less_than(std::string vehicle_name, size_t param, double value)
    : condition(vehicle_name, "less_than", param, value)
  {
  }

  /**
   * @brief Implementation of how the condition is handled.
   * @param BLKB Running blackboard
   */
  BT_Status update(blackboard *BLKB)
  {
    double data = BLKB->get(var.data());
    if (data < limit) { // Check whether the data in the condition is less than than the limit
      return BH_SUCCESS;
    } else {
      return BH_FAILURE;
    }
  }
};

}
#endif // LESS_THAN_H