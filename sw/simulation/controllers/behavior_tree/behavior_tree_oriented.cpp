#include "behavior_tree_oriented.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include <algorithm>
using namespace std;

#define SENSORS 4

behavior_tree_oriented::behavior_tree_oriented(): t(4)
{
  // Load the behavior tree
  tree = loadFile(param->policy().c_str());

  // Initialize input sensors
  string name;
  for (size_t i = 0; i < SENSORS; i++) {
    name = "sensor" + to_string(i);
    BLKB.set(name.c_str(), 0);
  }

  // Initialize outputs
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1
}

void behavior_tree_oriented::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;
  get_lattice_motion_all(ID, v_x, v_y);

  vector<bool> sensor;
  vector<int> temp;
  t.assess_situation(ID, sensor, temp);

  /**** Step 1 of 3: Set current state according to sensors ****/
  string name;
  for (size_t i = 0; i < sensor.size(); i++) {
    name = "sensor" + to_string(i);
    if (sensor[i]) {BLKB.set(name.c_str(), 1);}
    else {BLKB.set(name.c_str(), 0);}
  }

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  v_x += BLKB.get("wheelSpeed0");
  v_y += BLKB.get("wheelSpeed1");
  wall_avoidance_turn(ID, v_x, v_y);

  string d = to_string(v_x) + ", " +  to_string(v_y);
  terminalinfo::debug_msg(d, ID);
}

void behavior_tree_oriented::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}