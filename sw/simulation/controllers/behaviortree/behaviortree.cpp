#include "behaviortree.h"
#include "draw.h"

behaviortree::behaviortree() : Controller()
{
  // Load the behavior tree from the file
  tree = loadFile("/home/mario/repos/bt_evolution/behaviortree_temp/behaviortree.xml");
  cout << "Loaded the behavior tree successfully" << endl;
}

blackboard BB;
void behaviortree::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  /**** Step 1 of 3: Set current state according to sensors ****/
  // BB.set(0);

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BB);

  /**** Step 3 of 3: Set output ****/
  v_x = BB.get("wheelSpeed0");
  v_y = BB.get("wheelSpeed1");

  // Debug
  cout << int(ID) << " " << v_x << " " << v_y << endl;
}
