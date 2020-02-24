#include "behavior_tree.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"

// #define BEHAVIOR_TREE "/home/mario/repos/bt_evolution/behaviortree_temp/behaviortree.xml"
// #define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behavior_tree_aggregation.xml"
#define BEHAVIOR_TREE "/home/mario/repos/swarmulator/conf/behavior_trees/behaviortree_evolved_aggregation.xml"

#ifdef ARENAWALLS
void wall_avoidance(uint8_t ID, uint &walltimer, float &v_x_ref, float &v_y_ref)
{
  terminalinfo ti;
  float timelim = 2.0 * param->simulation_updatefreq();

  walltimer++;
  if (s[ID]->get_position(0) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 0;
    ti.debug_msg("Agent left the arena from the North side");
    v_x_ref = -abs(v_x_ref);
  }

  if (s[ID]->get_position(0) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 0;
    ti.debug_msg("Agent left the arena from the South side");
    v_x_ref = abs(v_x_ref);
  }

  if (s[ID]->get_position(1) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 0;
    ti.debug_msg("Agent left the arena from the East side");
    v_y_ref = -abs(v_y_ref);
  }

  if (s[ID]->get_position(1) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 0;
    ti.debug_msg("Agent left the arena from the West side");
    v_y_ref = abs(v_y_ref);
  }
}
#endif

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

void behavior_tree::lattice_all(const int &ID, const vector<uint> &closest, float &v_x, float &v_y)
{
  if (!closest.empty()) {
    for (size_t i = 0; i < closest.size(); i++) {
      get_lattice_motion(ID, closest[i], v_x, v_y);
    }
    v_x = v_x / (float)closest.size();
    v_y = v_y / (float)closest.size();
  }
}


behavior_tree::behavior_tree() : Controller()
{
  // Load the behavior tree
  tree = loadFile(BEHAVIOR_TREE);

  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);

  // Initialize input sensors
  BLKB.set("sensor0", 0.); // Sensor 0

  // Initialize outputs
  BLKB.set("wheelSpeed0", 0.); // Output 0
  BLKB.set("wheelSpeed1", 0.); // Output 1

  moving_timer = 0;
#ifdef ARENAWALLS
  walltimer = 0;
#endif
}

void behavior_tree::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  float timelim = 2.0 * param->simulation_updatefreq();

  // Get vector of all neighbors from closest to furthest

  vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
  lattice_all(ID, closest, v_x, v_y);

  float vmean = 1.0;

  /**** Step 1 of 3: Set current state according to sensors ****/
  BLKB.set("sensor0", closest.size());

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  float p_motion = BLKB.get("wheelSpeed0");

  terminalinfo ti;
  string d = "p=" + to_string(p_motion);
  ti.debug_msg(d, ID); // Debug

  /******** Probabilistic aggregation behavior ***********/
  // Initialize local moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rg.uniform_int(0, timelim);
  }
  // Behavior
  if (moving_timer == 1 && walltimer > 2.0 * timelim) {
    if (rg.bernoulli(1.0 - p_motion)) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      ang = rg.uniform_float(0.0, 2 * M_PI);
      if (moving) {
        float ext = rg.gaussian_float(0.0, 0.5);
        float temp;
        cart2polar(v_x_ref, v_y_ref, temp, ang);
        ang += ext;
      }
      wrapTo2Pi(ang);
      polar2cart(vmean, ang, v_x_ref, v_y_ref);
      moving = true;
    }
  }
  increase_counter(moving_timer, timelim);
  /*******************************************************/
#ifdef ARENAWALLS
  wall_avoidance(ID, walltimer, v_x_ref, v_y_ref);
#endif

  v_x += v_x_ref;
  v_y += v_y_ref;

  d = to_string(v_x) + ", " +  to_string(v_y);
  ti.debug_msg(d, ID);
}
