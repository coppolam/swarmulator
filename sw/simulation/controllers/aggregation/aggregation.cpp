#include "aggregation.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"
#include "draw.h"

using namespace std;
aggregation::aggregation() : Controller()
{
  // Initial values
  moving = false;
  moving_timer = rg.uniform_int(0, timelim);
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);

  // Control values
  timelim = 2.0 * param->simulation_updatefreq();
  vmean = 0.5;

  // Policy
  if (!strcmp(param->policy().c_str(), "")) { motion_p.assign(7, 0.); motion_p[0] = 1.; }
  else { motion_p = read_array(param->policy()); }
}

void aggregation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = std::min(r.size(), motion_p.size());
    if (rg.bernoulli(1.0 - motion_p[st])) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      float ang = rg.uniform_float(0.0, 2 * M_PI);
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
  increase_counter_to_value(moving_timer, timelim, 1);
  wall_avoidance_bounce(ID, v_x_ref, v_y_ref);

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}

void aggregation::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}