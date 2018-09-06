#include "controller.h"

#include <cmath>
#include <fstream>
#include <numeric>
#include <unistd.h>
#include <random>

#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

Controller::Controller()
{
  _ddes = 1.0;
  _ddes_x = 1.5;
  _ddes_y = 1.0;
  _kr = 1.0;
  _ka = 5;
  saturation = false;
};

Controller::~Controller() {};

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