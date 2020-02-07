#include "blackboard.h"

// read data identified by string var from blackboard data manager
double blackboard::get(const char *var, const int k /*= -1*/)
{
  char name [255];
  if (k >= 0) {
    sprintf(name, "%s%d", var, k);
  } else {
    strcpy(name, var);
  }

  if (blackboard::BB.find(name) == blackboard::BB.end()) {
    printf("get Requested element does not exist in the blackboard: %s %s\n", var, name);
    return 0.;
  }

  return blackboard::BB[name];
}

// write data identified by string var to blackboard data manager
void blackboard::set(const char *var, double data, const int k/* = -1*/)
{
  char name [255];
  if (k >= 0) {
    sprintf(name, "%s%d", var, k);
  } else {
    strcpy(name, var);
  }

  blackboard::BB[name] = data;
}
