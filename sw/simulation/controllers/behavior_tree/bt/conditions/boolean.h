#ifndef BOOLEAN_H
#define BOOLEAN_H

#include "condition.h"

namespace BT
{
/**
 * Class to check the great_than condition
 */
class boolean : public condition
{
public:
  /**
   * @brief Constructor 1 for a new boolean object
   */
  boolean(std::string vehicle_name, size_t param = MAX_SIZE)
    : condition(vehicle_name, "boolean", param)
  {
  }
  /**
   * @brief Constructor 2 for a new boolean object
   */
  boolean(std::string vehicle_name, size_t param, bool value)
    : condition(vehicle_name, "boolean", param, value)
  {
  }

  BT_Status update(blackboard *BLKB)
  {
    bool data = BLKB->get(var.data());
    // std::cout << data << std::endl;
    if (data) { // Check whether the data in the condition is greater than than the limit
      return BH_SUCCESS;
    } else {
      return BH_FAILURE;
    }
  }
};

}

#endif // BOOLEAN_H