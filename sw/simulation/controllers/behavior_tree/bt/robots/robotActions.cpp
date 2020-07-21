#include "robotActions.h"

// #include "wheelSpeed.h"
#include "decision.h"

size_t KACTION = 1;       // total number of actions

namespace BT
{

BT_Status decision::update(blackboard *BLKB)
{
  // BLKB->set("wheelSpeed0", leftWheelSpeed);
  // BLKB->set("wheelSpeed1", rightWheelSpeed);
  BLKB->set("decision", decision_probability);
  return BH_SUCCESS;
}

// Get action, when inputs are known
node *getAction(std::string action, std::vector<double> inputs /*= NULL*/)
{
  // node *task;
  // if (inputs.empty()) {
  //   if (action.compare("wheelSpeed") == 0) {
  //     task = (node *) new wheelSpeed;
  //   } else {
  //     perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
  //   }
  // } else {
  //   if (action.compare("wheelSpeed") == 0) {
  //     task = (node *) new wheelSpeed(inputs[0], inputs[1]);
  //   } else {
  //     perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
  //   }
  // }
  node *task;
  if (inputs.empty()) {
    if (action.compare("decision") == 0) {
      task = (node *) new decision;
    } else {
      perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
    }
  } else {
    if (action.compare("decision") == 0) {
      task = (node *) new decision(inputs[0]);
    } else {
      perror("Something is really wrong in :BT::Composite* getAction(std::string action, std::vector<double>* inputs)");
    }
  }

  return task;
}

// Get action, when inputs are known
// Add all actions to the if else if list below
node *getAction(size_t func /*= (size_t) - 1*/)
{
  node *task;
  switch (func) {
    // case 0:
    //   task = (node *) new wheelSpeed;
    //   break;
    case 0:
      task = (node *) new decision;
      break;
    default:
      //std::cerr << "ERROR in getAction(unsigned int func): number of actions out of bounds"<<std::endl;
      return getAction(rand() % KACTION);
  }
  return task;
}

}
