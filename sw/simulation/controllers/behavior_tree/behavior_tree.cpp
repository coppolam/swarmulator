#include "behavior_tree.h"
#include "draw.h"

behaviortree::behaviortree() : Controller()
{
  // Load the behavior tree from the file
  tree = loadFile("./conf/behaviortrees/behaviortree_test.xml");
  cout << "Loaded the behavior tree successfully" << endl;
  BLKB.set("wheelSpeed0",0.);
  BLKB.set("wheelSpeed1",0.);
  BLKB.set("sensor0", 0. );
}


void behaviortree::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  /**** Step 1 of 3: Set current state according to sensors ****/
  BLKB.set("sensor0",0.01);

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set output ****/
  v_x = BLKB.get("wheelSpeed0");
  v_y = BLKB.get("wheelSpeed1");

  // Debug
  cout << int(ID) << " " << v_x << " " << v_y << endl;
}
