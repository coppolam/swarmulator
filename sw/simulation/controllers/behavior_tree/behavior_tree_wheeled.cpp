#include "behavior_tree_wheeled.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include <algorithm>

#define BEHAVIOR_TREE "/home/mario/repos/bt_evolution/behaviortree_temp/behaviortree.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behavior_tree_aggregation.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behaviortree_evolved_aggregation.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behavior_tree_wheeled_disperse.xml"

#define KNEAREST 6

// Defining the binary function
bool comp(int a, int b)
{
  return (a < b);
}

behavior_tree_wheeled::behavior_tree_wheeled() : Controller()
{
  // Load the behavior tree
  tree = loadFile(BEHAVIOR_TREE);

  // Initialize input sensors
  string name;
  for (size_t i = 0; i < KNEAREST; i++) {
    name = "sensor" + to_string(i);
    BLKB.set(name.c_str(), 0.);
  }

  // Initialize outputs
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1
}

void behavior_tree_wheeled::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1
  
  float timelim = 2.0 * param->simulation_updatefreq();

  vector<float> r, b;
  o.request_relative_location_inrange(ID, rangesensor, r, b);

  /**** Step 1 of 3: Set current state according to sensors ****/
  if (b.size() > 0) {
    string name;
    for (size_t i = 0; i < min((uint)r.size(), (uint)KNEAREST); i++) {
      name = "sensor" + to_string(i);
      BLKB.set(name.c_str(), b[i]); // bearing of agent i
    }
  }

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  
  float wheelSpeedleft = BLKB.get("wheelSpeed0");
  float wheelSpeedright = BLKB.get("wheelSpeed1");

  /******** Probabilistic aggregation behavior ***********/
  // Initialize local moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rg.uniform_int(0, timelim);
  }
  // Behavior
  else {
    v_x += wheelSpeedleft;
    v_y += wheelSpeedright;
  }
  increase_counter(moving_timer, timelim);

  string d = to_string(v_x) + ", " +  to_string(v_y);
  terminalinfo::debug_msg(d, ID);
}
