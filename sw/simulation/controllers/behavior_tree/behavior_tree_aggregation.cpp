#include "behavior_tree_aggregation.h"
#include "draw.h"
#include "terminalinfo.h"
#include "auxiliary.h"

using namespace std;

behavior_tree_aggregation::behavior_tree_aggregation() : Controller()
{
  // Load and initialize the behavior tree
  tree = loadFile(param->policy().c_str());
  BLKB.set("sensor0", 0); // Initialize input 0
  BLKB.set("decision", 0.5); // Initialize output 0

  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);
  timelim = 2.0 * param->simulation_updatefreq();
  moving_timer = rg.uniform_int(0, timelim);
}

void behavior_tree_aggregation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  // Get vector of all neighbors from closest to furthest

  vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  float vmean = 1.0;

  /**** Step 1 of 3: Set current state according to sensors ****/
  BLKB.set("sensor0", closest.size());

  /**** Step 2 of 3: Tick the tree based on the current state ****/
  tree->tick(&BLKB);

  /**** Step 3 of 3: Set outputs (do this once, else keep!) ****/
  float p_motion = BLKB.get("decision");

  string d = "p=" + to_string(p_motion);
  terminalinfo::debug_msg(d, ID); // Debug

  /******** Probabilistic aggregation behavior ***********/
  if (moving_timer == 1) {
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

  wall_avoidance_bounce(ID, v_x_ref, v_y_ref);

  v_x += v_x_ref;
  v_y += v_y_ref;

  d = to_string(v_x) + ", " +  to_string(v_y);
  terminalinfo::debug_msg(d, ID);
}


void behavior_tree_aggregation::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}