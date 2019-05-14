#include "controller_cartesian.h"
#include "agent.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"

float Controller_Cartesian::f_attraction(float u)
{
  //% Sigmoid function -- long-range
  float ddes = 1.5;
  float w = log((ddes / _kr - 1) / exp(-_ka * ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}

float Controller_Cartesian::get_attraction_velocity(float u)
{
  return f_attraction(u) + f_repulsion(u);
}

void Controller_Cartesian::get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y)
{
  float v_b, v_r;
  v_b = wrapToPi_f(o.request_bearing(ID, state_ID));
  v_r = get_attraction_velocity(o.request_distance(ID, state_ID));
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}


void Controller_Cartesian::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;

  float timelim = 2.0 * param->simulation_updatefreq();
  // Initialize local moving_timer with random variable
  if (moving_timer == 0) {
    moving_timer = rg.uniform_int(0, timelim);
  }

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

  if (moving_timer == 1 && walltimer > 2 * timelim) {
    if (rg.bernoulli(1.0 - motion_p[q_ID.size()])) {
      v_x_ref = 0.0;
      v_y_ref = 0.0;
      moving = false;
    } else { // Else explore randomly, change heading
      ang = rg.uniform_float(0.0,2*M_PI);
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

#ifdef CHECK_HAPPY
  if (q_ID.size() > 1) {
    happy = true;
  } else {
    happy = false;
  }
#endif

#ifdef ARENAWALLS
  walltimer++;
  if (s[ID]->get_position(0) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
    v_x_ref = -vmean;
  }

  if (s[ID]->get_position(0) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
    v_x_ref = vmean;
  }

  if (s[ID]->get_position(1) > ARENAWALLS / 2.0 - rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
    v_y_ref = -vmean;
  }

  if (s[ID]->get_position(1) < -ARENAWALLS / 2.0 + rangesensor && walltimer > 2 * timelim) {
    walltimer = 1;
    v_y_ref = vmean;
  }
#endif

  if (moving_timer > timelim) {
    moving_timer = 0;
  }
  moving_timer++;

  v_x += v_x_ref;
  v_y += v_y_ref;
}
