#include "behavior_tree.h"
#include "draw.h"

behavior_tree::behavior_tree() : Controller()
{
  // Load the behavior tree from the file
  tree = loadFile("./conf/behavior_trees/behavior_tree_example.xml");
  cout << "Loaded the behavior tree successfully" << endl;
  BLKB.set("wheelSpeed0", 0.);
  BLKB.set("wheelSpeed1", 0.);
  BLKB.set("sensor0", 0.5);
}


void behavior_tree::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  /**** Step 1 of 3: Set current state according to sensors ****/
  BLKB.set("sensor0", 0.5);

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set output ****/
  v_x = BLKB.get("wheelSpeed0");
  v_y = BLKB.get("wheelSpeed1");

  // Debug
  cout << "Robot " << int(ID) << ": \t" << v_x << ", " << v_y << endl;
}
