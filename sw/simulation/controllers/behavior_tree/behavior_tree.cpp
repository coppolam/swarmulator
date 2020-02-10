#include "behavior_tree.h"
#include "draw.h"

#define BEHAVIOR_TREE "/home/mario/repos/bt_evolution/behaviortree_temp/behaviortree.xml"

float behavior_tree::f_attraction(float u)
{
  //% Sigmoid function -- long-range
  float ddes = 1.5;
  float w = log((ddes / _kr - 1) / exp(-_ka * ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}

float behavior_tree::get_attraction_velocity(float u)
{
  return f_attraction(u) + f_repulsion(u);
}

void behavior_tree::get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y)
{
  float v_b, v_r;
  v_b = wrapToPi_f(o.request_bearing(ID, state_ID));
  v_r = get_attraction_velocity(o.request_distance(ID, state_ID));
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}

behavior_tree::behavior_tree() : Controller()
{
  // Load the behavior tree
  tree = loadFile(BEHAVIOR_TREE);
  
  // Initialize input sensors
  BLKB.set("sensor0", 0.); // Sensor 0
  
  // Initialize outputs
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1

  #ifdef ARENAWALLS
  walltimer = 1;
  #endif
}

void behavior_tree::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{  
  v_x = 0;
  v_y = 0;
  
  float timelim = 2.0 * param->simulation_updatefreq();

  // Get vector of all neighbors from closest to furthest
  vector<int> closest = o.request_closest(ID);
  vector<int> q_ID;
  q_ID.clear();
  for (uint8_t i = 0; i < nagents - 1; i++) {
    if (o.request_distance(ID, closest[i]) < rangesensor) {
      q_ID.push_back(closest[i]); // Log ID (for simulation purposes only, depending on assumptions)
    }
  }

  if (!q_ID.empty()) {
    for (size_t i = 0; i < q_ID.size(); i++) {
      get_lattice_motion(ID, q_ID[i], v_x, v_y);
    }
    v_x = v_x / (float)q_ID.size();
    v_y = v_y / (float)q_ID.size();
  }
  float vmean = 0.5;

  /**** Step 1 of 3: Set current state according to sensors ****/
  BLKB.set("sensor0", 0.5);

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB, &tickID);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  float v_x_ref = BLKB.get("wheelSpeed0");
  float v_y_ref = BLKB.get("wheelSpeed1");

// #ifdef ARENAWALLS
  walltimer++;
  if (s[ID]->get_position(0) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
    cout << "out " << 1 << endl;
    v_x_ref = -vmean;
  }

  if (s[ID]->get_position(0) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
        cout << "out " <<2 << endl;
    v_x_ref = vmean;
  }

  if (s[ID]->get_position(1) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
        cout << "out " << 3 << endl;

    v_y_ref = -vmean;
  }

  if (s[ID]->get_position(1) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
        cout << "out " << 4 << endl;

    v_y_ref = vmean;
  }
// #endif

  v_x += v_x_ref;
  v_y += v_y_ref;

  // Debug
  // cout << "Robot " << int(ID) << ": \t" << v_x << ", " << v_y << endl;
}
