#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <algorithm>
#include <stdint.h>
#include "omniscient_observer.h"

using namespace std;

class Controller
{
  
public:
  Controller();
  ~Controller();

  float _kr = 1; // Repulsion gain
  float _ddes = 1.0; // Desired equilibrium distance
  float _ka = 5; // Attraction gain

  bool  saturation;
  float saturation_limits;

  void set_saturation(const float &saturation_limits);
  float saturate(float f);
  float f_repulsion(float u);

  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y) = 0;
};


#endif /*CONTROLLER_H*/
