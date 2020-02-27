#include "behavior_tree_lf.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include <algorithm>

#define BEHAVIOR_TREE "/home/mario/repos/bt_evolution/behaviortree_temp/behaviortree.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behavior_tree_aggregation.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behaviortree_evolved_aggregation.xml"

#define KNEAREST 6

behavior_tree_lf::behavior_tree_lf() : Controller()
{
  // Load the behavior tree
  tree = loadFile(BEHAVIOR_TREE);
  
  // Initialize input sensors
  BLKB.set("sensor0",0.0); // Relative px
  BLKB.set("sensor1",0.0); // Relative px
  BLKB.set("sensor2",0.0); // Relative px
  BLKB.set("sensor3",0.0); // Relative px

  // Initialize outputs
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1
}

void behavior_tree_lf::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  // Leader is trying not to be followed
  if (ID == 0) {
    v_x = 1.0;
    v_y = 0.0;
  }

  // Follower is trying to follower the leader
  else {
    uint8_t ID_tracked = 0; //ID - 1;
    filter.run(ID, ID_tracked);
    BLKB.set("sensor0",filter.ekf_rl.X[0]); // Relative px
    BLKB.set("sensor1",filter.ekf_rl.X[1]); // Relative py
    BLKB.set("sensor2",filter.ekf_rl.X[4]); // Relative vx
    BLKB.set("sensor3",filter.ekf_rl.X[5]); // Relative vy
    
    /**** Step 2 of 3: Tick the tree based on the current state ****/
    tree->tick(&BLKB);

    /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
    float wheelSpeedleft = BLKB.get("wheelSpeed0");
    float wheelSpeedright = BLKB.get("wheelSpeed1");
    v_x += wheelSpeedleft;
    v_y += wheelSpeedright;
  }

  string d = to_string(v_x) + ", " +  to_string(v_y);
  terminalinfo::debug_msg(d, ID);
}
