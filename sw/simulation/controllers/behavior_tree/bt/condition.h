#ifndef CONDITIONS_H
#define CONDITIONS_H

#include <stdlib.h>
#include <tuple>
#include "node.h"
#include "settings.h"

#define MAX_SIZE (size_t) - 1
#define KCOND 1
#define NUMBER_OF_VARS 6 // Total number of variables = sensors!


/**
 * @brief Behavior tree namespace
 *
 * Holds the behavior tree structure.
 */
namespace BT
{

std::tuple<std::string, double, double, double> input(int i);

node *getCondition(std::string condition, size_t var = MAX_SIZE);
node *getCondition(std::string condition, std::vector<double> inputs);
node *getCondition(size_t func = rand() % KCOND, size_t var = MAX_SIZE);

class condition : public node
{
public:
  condition(std::string vehicle_name, std::string func_name, size_t param, double value)
    : node(vehicle_name, "condition", func_name),
      k_var(param),
      limit(value)
  {
    std::tuple<std::string, double, double, double> set = input(k_var);
    var = std::get<0>(set);

    vars.push_back(double(k_var));
    vars_lower_lim.push_back(double(k_var));
    vars_upper_lim.push_back(double(k_var));

    vars.push_back(limit);
    vars_lower_lim.push_back(std::get<2>(set));
    vars_upper_lim.push_back(std::get<3>(set));
  }
  condition(std::string vehicle_name, std::string func_name, size_t param = MAX_SIZE)
    : node(vehicle_name, "condition", func_name)
  {
    if (param >= NUMBER_OF_VARS) {
      k_var = rand() % NUMBER_OF_VARS;
    } else {
      k_var = param;
    }

    std::tuple<std::string, double, double, double> set = input(k_var);

    var = std::get<0>(set);
    limit = std::get<1>(set);

    vars.push_back(double(k_var));
    vars_lower_lim.push_back(double(k_var));
    vars_upper_lim.push_back(double(k_var));

    vars.push_back(limit);
    vars_lower_lim.push_back(std::get<2>(set));
    vars_upper_lim.push_back(std::get<3>(set));
  }

  size_t k_var;   // Parameter number
  std::string var;  // Parameter name
  double limit;   // threshold
};

}
#endif // CONDITIONS_H
