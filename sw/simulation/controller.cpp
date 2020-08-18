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
  _kr = 1.0; // Repulsion gain
  _ka = 0.1; // Attraction gain
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
  return _ka * u;
}

float Controller::f_attraction_equilibrium(float u, float eq_distance)
{
  // Sigmoid function -- long-range
  float w = log(abs(eq_distance / _kr - 1.0) / exp(-_ka * eq_distance)) / _ka;
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

void Controller::get_lattice_motion_range(const int &ID, float &v_x, float &v_y, float rangemax)
{
  std::vector<uint> closest = o.request_closest(ID);
  std::vector<uint> q_ID;
  q_ID.clear();
  for (uint16_t i = 0; i < s.size() - 1; i++) {
    if (o.request_distance(ID, closest[i]) < rangemax) {
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

void Controller::get_lattice_motion_k_nearest(const int &ID, float &v_x, float &v_y, uint8_t k)
{
  std::vector<uint> closest = o.request_closest(ID);
  if (!closest.empty()) {
    uint mm = std::min(int(closest.size()), int(k));
    for (size_t i = 0; i < mm; i++) {
      get_lattice_motion(ID, closest[i], v_x, v_y);
    }
    v_x += v_x / (float)k;
    v_y += v_y / (float)k;
  }
}


float Controller::get_attraction_velocity(float u)
{
  return f_attraction_equilibrium(u, 1.5) + f_repulsion(u);
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

bool Controller::wall_avoidance_bounce(const uint16_t ID, float &v_x, float &v_y, float rangesensor)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  std::vector<float> sn = s[ID]->state;
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
    return true;
  }
  return false;
}

bool Controller::wall_avoidance_turn(const uint16_t ID, float &v, float &dpsitheta, float rangesensor)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  std::vector<float> sn = s[ID]->state;
  float r_temp, ang_temp, vx_temp, vy_temp, vx_global, vy_global, slope;
  rotate_xy(0.5, 0.5, sn[6], vx_global, vy_global);
  cart2polar(vx_global, vy_global, r_temp, ang_temp); // direction of velocity
  polar2cart(rangesensor, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  sn[0] += vx_temp;
  sn[1] += vy_temp;
  bool test1 = environment.sensor(ID, sn, s[ID]->state, slope);

  sn = s[ID]->state;
  rotate_xy(0.5, -0.5, sn[6], vx_global, vy_global);
  cart2polar(vx_global, vy_global, r_temp, ang_temp); // direction of velocity
  polar2cart(rangesensor, ang_temp, vx_temp, vy_temp); // use rangesensor to sense walls
  sn[0] += vx_temp;
  sn[1] += vy_temp;

  bool test2 = environment.sensor(ID, sn, s[ID]->state, slope);
  if (test1 || test2) {
    v = 0.;
    dpsitheta = 0.4;
    return true; // Wall!
  }
  return false; // No wall
}

float Controller::get_max_sensor_range()
{
  return sensor_range_max;
}
void Controller::set_max_sensor_range(float r)
{
  sensor_range_max = r;
}