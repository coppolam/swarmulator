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

float Controller::f_repulsion(float u)
{
  return -_kr / u;
}

void Controller::set_saturation(const float &lim)
{
  saturation = true;
  saturation_limits = lim;
}

void Controller::wall_avoidance(uint8_t ID, float &v_x, float &v_y)
{
  // Predict what the command wants and see if it will hit a wall, then fix it.
  vector<float> sn(2);
  float margin = 2.;
  sn[0] = s[ID]->state[0] + margin*v_x;
  sn[1] = s[ID]->state[1] + margin*v_y;
  bool test = environment.sensor(ID, sn, s[ID]->state);
  if (test) {
    float v, ang;
    cart2polar(v_x, v_y, v, ang);
    polar2cart(v, wrapTo2Pi_f(ang + M_PI), v_x, v_y);
  }
}