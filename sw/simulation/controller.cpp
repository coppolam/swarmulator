#include "controller.h"

#include <cmath>
#include <fstream>
#include <numeric>
#include <unistd.h>
#include <random>

#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

Controller::Controller() {};
Controller::~Controller() {};

float Controller::saturate(float f)
{
  if (saturation) {
    keepbounded(f, -saturation_limits, saturation_limits);
  }
  return f;
}

void Controller::set_saturation(const float &lim)
{
  saturation = true;
  saturation_limits = lim;
}