#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <string>
#include "string.h"
#include <map>
/*
 * blackboard
 * Used to store all data for the Behaviour Tree to access
 * get() - returns the value in the blackboard with name "var"
 *      for multiple entries with the same name k is used as an iterator
 *      throws an error if element does not exist in list
 * set() - sets the value of the variable named "var"
 *      if var is not in the list it is added
 *      for multiple entries with the same name k is used as an iterator
 */
struct blackboard {
public:
  blackboard() {}
  ~blackboard() {}
  double get(const char *var, const int k = -1);
  void set(const char *var, double data, const int k = -1);
private:
  std::map< std::string,  double > BB;
};

#endif // BLACKBOARD_H
