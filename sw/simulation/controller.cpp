#include "controller.h"

#include <cmath>
#include <fstream>
#include <numeric>
#include <unistd.h>

#include "main.h"
#include "randomgenerator.h"
#include "trigonometry.h"
#include "auxiliary.h"

Controller::Controller()
{
  _ddes_x = 1.0; // Desired distance at North
  _ddes_y = 1.0; // Desired distance at East
  _kr = 1.0;     // Repulsion gain
  _ka = 5.0;     // Attraction gain
  saturation = false; // Controller saturation
};

void Controller::saturate(float &f)
{
  if (saturation) {
    keepbounded(f, -saturation_limits, saturation_limits);
  }
}

float Controller::f_attraction(float u)
{
  //% Sigmoid function -- long-range
  float ddes = 1.5;
  float w = log((ddes / _kr - 1) / exp(-_ka * ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}


void Controller::get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y)
{
  float v_b, v_r;
  v_b = wrapToPi_f(o.request_bearing(ID, state_ID));
  v_r = get_attraction_velocity(o.request_distance(ID, state_ID));
  v_x += v_r * cos(v_b);
  v_y += v_r * sin(v_b);
}

void Controller::get_lattice_motion_all(const int &ID, float &v_x, float &v_y)
{
  vector<uint> closest = o.request_closest(ID);
  vector<uint> q_ID;
  q_ID.clear();
  for (uint8_t i = 0; i < s.size() - 1; i++) {
    if (o.request_distance(ID, closest[i]) < rangesensor) {
      q_ID.push_back(closest[i]); // Log ID (for simulation purposes only, depending on assumptions)
    }
  }
  if (!q_ID.empty()) {
    for (size_t i = 0; i < q_ID.size(); i++) {
      get_lattice_motion(ID, q_ID[i], v_x, v_y);
    }
    v_x += v_x / (float)q_ID.size();
    v_y += v_y / (float)q_ID.size();
  }
}


float Controller::get_attraction_velocity(float u)
{
  return f_attraction(u) + f_repulsion(u);
}

float Controller::f_repulsion(float u)
{
  return -_kr / u;
}

void Controller::set_saturation(const float &lim)
{
  saturation = true;
  saturation_limits = lim;
}

void Controller::wall_avoidance(const uint8_t ID, float &v_x, float &v_y)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  vector<float> sn = s[ID]->state;
  float r_temp, ang_temp, vx_temp, vy_temp;
  cart2polar(v_x, v_y, r_temp, ang_temp); // direction of velocity
  polar2cart(rangesensor, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  sn[0] += vx_temp;
  sn[1] += vy_temp;
  float slope;
  bool test = environment.sensor(ID, sn, s[ID]->state, slope);
  if (test) {
    float v, ang;
    cart2polar(v_x, v_y, v, ang);
    ang = rg.uniform_float(0, 2 * M_PI);
    polar2cart(v, ang, v_x, v_y);
    moving = false;
  }
}

void Controller::wall_avoidance_t(const uint8_t ID, float &v, float &dpsitheta)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  vector<float> sn = s[ID]->state;
  float r_temp, ang_temp, vx_temp, vy_temp, vx_global, vy_global, slope;
  rotate_xy(0.5, 0.2, sn[6], vx_global, vy_global);
  cart2polar(vx_global, vy_global, r_temp, ang_temp); // direction of velocity
  polar2cart(2.5, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  sn[0] += vx_temp;
  sn[1] += vy_temp;

  bool test1 = environment.sensor(ID, sn, s[ID]->state, slope);

  sn = s[ID]->state;
  rotate_xy(0.5, -0.2, sn[6], vx_global, vy_global);
  cart2polar(vx_global, vy_global, r_temp, ang_temp); // direction of velocity
  polar2cart(2.5, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  sn[0] += vx_temp;
  sn[1] += vy_temp;

  bool test2 = environment.sensor(ID, sn, s[ID]->state, slope);
  if (test1 || test2) {
    v = 0.;
    dpsitheta = 0.11;
    // cout << "Wallll" << endl;
  }
}
