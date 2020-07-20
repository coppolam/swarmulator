#include "behavior_tree_wheeled.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include <algorithm>
using namespace std;

#define KNEAREST 6

// Defining the binary function
bool comp(int a, int b)
{
  return (a < b);
}

behavior_tree_wheeled::behavior_tree_wheeled() : Controller()
{
  // Load the behavior tree
  tree = loadFile(param->policy().c_str());

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

void behavior_tree_wheeled::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  /**** Step 1 of 3: Set current state according to sensors ****/
  if (r.size() > 0) {
    string name;
    for (size_t i = 0; i < std::min((uint)r.size(), (uint)KNEAREST); i++) {
      name = "sensor" + to_string(i);
      BLKB.set(name.c_str(), r[i]);
    }
  }

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  float wheelSpeedleft = BLKB.get("wheelSpeed0");
  float wheelSpeedright = BLKB.get("wheelSpeed1");

  v_x += wheelSpeedleft;
  v_y += wheelSpeedright;

  string d = to_string(v_x) + ", " +  to_string(v_y);
  terminalinfo::debug_msg(d, ID);
}

void behavior_tree_wheeled::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}