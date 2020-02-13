#ifndef MEMORY_SET_H
#define MEMORY_SET_H

#include "condition.h"

namespace BT
{

class memory_set : public condition
{
public:
  memory_set(std::string vehicle_name, size_t param = MAX_SIZE)
    : condition(vehicle_name, "memory_set", param)
  {
  }
  memory_set(std::string vehicle_name, size_t param, double value)
    : condition(vehicle_name, "memory_set", param, value)
  {
  }

  BT_Status update(blackboard *BLKB)
  {
    if (BLKB->get("memory")) {
      return BH_SUCCESS;
    } else {
      return BH_FAILURE;
    }

  }
};

}

#endif // MEMORY_SET_H