#include "robotActions.h"

size_t KACTION = 1;       // total number of actions

namespace BT
{

BT_Status wheelSpeed::update(blackboard *BLKB)
{
  BLKB->set("wheelSpeed0", leftWheelSpeed);
  BLKB->set("wheelSpeed1", rightWheelSpeed);

  return BH_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////////////////
node *getAction(std::string action, std::vector<double> inputs /*= NULL*/)
{
  node *task;
  if (inputs.empty()) {
    if (action.compare("wheelSpeed") == 0) {
      task = (node *) new wheelSpeed;
    } else {
      perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
    }
  } else {
    if (action.compare("wheelSpeed") == 0) {
      task = (node *) new wheelSpeed(inputs[0], inputs[1]);
    } else {
      perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
    }
  }

  return task;
}

// Add all actions to the if else if list below
node *getAction(size_t func /*= (size_t) - 1*/)
{
  node *task;
  switch (func) {
    case 0:
      task = (node *) new wheelSpeed;
      break;
    default:
      //std::cerr << "ERROR in getAction(unsigned int func): number of actions out of bounds"<<std::endl;
      return getAction(rand() % KACTION);
  }
  return task;
}

}
