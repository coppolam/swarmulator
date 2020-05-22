#include "controller_aggregation.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

controller_aggregation::controller_aggregation() : Controller()
{
  moving = false;
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);
  string p = param->policy();
  if (!strcmp(p.c_str(), "")) {
    motion_p = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    // motion_p = {0.991355, 0.984845, 0.007304, 0.000783, 0.004238, 0.001033, 0.007088};
    // motion_p = {1.0, 0.0, 0.0, 0.0, 1.0, 0.14, 0.14};
    // motion_p = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  } else {
    motion_p = read_array(p);
  }
  timelim = 2.0 * param->simulation_updatefreq();
  moving_timer = rg.uniform_int(0, timelim);
  vmean = 0.5;
}

void controller_aggregation::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  get_lattice_motion_all(ID, v_x, v_y); // Repulsion from neighbors

  // Sense neighbors
  vector<float> r, b;
  o.relative_location_inrange(ID, rangesensor, r, b);

  if (st != r.size() || moving_timer == 1) { // state change
    // state, action
    st = min(r.size(), motion_p.size());
#ifdef ESTIMATOR
    int a;
    if (moving) {a = 1;} else {a = 0;}
    pr.update(ID, st, a); // pr update
#endif
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
  wall_avoidance(ID, v_x_ref, v_y_ref);

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}
