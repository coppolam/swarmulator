#include "controller_aggregation.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "auxiliary.h"

controller_aggregation::controller_aggregation() : Controller()
{
  moving = false;
  v_x_ref = rg.gaussian_float(0.0, 1.0);
  v_y_ref = rg.gaussian_float(0.0, 1.0);
  // motion_p = {P1, P2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  motion_p = {0.991355, 0.984845, 0.007304, 0.000783, 0.004238, 0.001033, 0.007088};
  moving_timer = 0;
  vmean = 0.5;
}

float controller_aggregation::f_attraction(float u)
{
  //% Sigmoid function -- long-range
  float ddes = 1.5;
  float w = log((ddes / _kr - 1) / exp(-_ka * ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}

float controller_aggregation::get_attraction_velocity(float u)
{
  return f_attraction(u) + f_repulsion(u);
}

void controller_aggregation::get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y)
{
  float v_b, v_r;
  v_b = wrapToPi_f(o.request_bearing(ID, state_ID));
  v_r = get_attraction_velocity(o.request_distance(ID, state_ID));
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}


void controller_aggregation::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  float timelim = 2.0 * param->simulation_updatefreq();
  // Initialize local moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rg.uniform_int(0, timelim);
  }

  // Get vector of all neighbors from closest to furthest
  vector<uint> closest = o.request_closest(ID);
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

  if (moving_timer == 1) {
    if (rg.bernoulli(1.0 - motion_p[q_ID.size()])) {
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
  increase_counter(moving_timer, timelim);

#ifdef CHECK_HAPPY
  if (q_ID.size() > 1) {
    happy = true;
  } else {
    happy = false;
  }
#endif

  wall_avoidance(ID, v_x_ref, v_y_ref);

  // Final output
  v_x += v_x_ref;
  v_y += v_y_ref;
}

void controller_aggregation::wall_avoidance(uint8_t ID, float &v_x, float &v_y)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  vector<float> sn(2);
  sn[0] = s[ID]->state[0] + v_x;
  sn[1] = s[ID]->state[1] + v_y;
  float ang;
  bool test = environment.sensor(ID, sn, s[ID]->state, ang);
  if (test) {
    float v, ang;
    cart2polar(v_x, v_y, v, ang);
    polar2cart(vmean, wrapTo2Pi_f(ang + M_PI), v_x, v_y);
  }
}