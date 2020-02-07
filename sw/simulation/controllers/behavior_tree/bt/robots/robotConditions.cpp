#include "../conditions.h"

size_t KCOND = 2;           //total number of conditions
size_t NUMBER_OF_VARS = 10; // total number of variables

namespace BT
{

std::tuple<std::string, double, double, double> input(int i)
{
  char buff[255];
  double threshold;
  double lower_lim;
  double upper_lim;

  if (i < 8) {
    sprintf(buff, "sensor%d", i);
    lower_lim = 0.;
    upper_lim = 1000;
  } else if (i < 10) {
    sprintf(buff, "wheelSpeed%d", i - 8);
    lower_lim = -0.5;
    upper_lim = 0.5;
  } else {
    std::cerr << "check number of variables, requested: " << i << std::endl;
    sprintf(buff, "sensor");
    lower_lim = 0.;
    upper_lim = 1000;
  }

  threshold = (upper_lim - lower_lim) * (rand() % 101) / 100. +
              lower_lim;            // test for 100 discrete points on range [lower_lim, upper_lim]

  return std::make_tuple(buff, threshold, lower_lim, upper_lim);
}

// ***************************************************************
// ***************************************************************
// Add all conditions to the if else if list below

node *getCondition(std::string condition, size_t var)
{
  node *task;
  if (condition.compare("less_than") == 0) {
    task = (node *) new less_than("robot", var);
  } else if
  (condition.compare("greater_than") == 0) {
    task = (node *) new greater_than("robot", var);
  } else {
    task = NULL;
    std::cerr << "Something is really wrong in :BT::node* getCondition(std::string condition)" << std::endl;
  }

  return task;
}

node *getCondition(std::string condition, std::vector<double> inputs)
{
  node *task;
  if (condition.compare("less_than") == 0) {
    task = (node *) new less_than("robot", static_cast<size_t>(inputs[0]), inputs[1]);
  } else if (condition.compare("greater_than") == 0) {
    task = (node *) new greater_than("robot", static_cast<size_t>(inputs[0]), inputs[1]);
  } else {
    task = NULL;
    std::cerr << "Something is really wrong in :BT::node* getCondition(std::string condition, std::vector<double>* inputs)"
              << std::endl;
  }

  return task;
}

// Add all conditions to the if else if list below
node *getCondition(size_t func /*= rand() % KCOND*/, size_t var /*= MAX_SIZE*/)
{
  node *task;
  switch (func) {
    case 0:
      task = (node *) new greater_than("robot", var);
      break;
    case 1:
      task = (node *) new less_than("robot", var);
      break;
    default:
      std::cerr << "ERROR in getCondition(unsigned int func): number of conditions out of bounds" << std::endl;
  }
  return task;
}

}
