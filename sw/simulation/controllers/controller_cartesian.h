#ifndef CONTROLLER_CARTESIAN_H
#define CONTROLLER_CARTESIAN_H

#include "controller.h"
#include "randomgenerator.h"
#include "trigonometry.h"

using namespace std;

/*
 * This controller handles attraction and velocity in North and East separately
 */
class Controller_Cartesian: public Controller
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;

public:
  bool moving;
  float v_x_ref, v_y_ref, ang;
  vector<float> motion_p;
  int moving_timer;
  int walltimer;
  random_generator rg;

  /**
   * Construction. Controller_Cartesian is a child class of Controller.
   */
  Controller_Cartesian() : Controller()
  {
    moving = false;
    v_x_ref = rg.gaussian_float(0.0, 1.0);
    v_y_ref = rg.gaussian_float(0.0, 1.0);
    float r;
    cart2polar(v_x_ref, v_y_ref, r, ang);
    // motion_p = {0.991355, 0.984845, 0.007304, 0.000783, 0.004238, 0.001033, 0.007088};
    // motion_p = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // motion_p = {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // motion_p = {1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    motion_p = {0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    moving_timer = 0;
    walltimer = 0;
  };

  /**
   * Attraction function at distance u
   */
  float f_attraction(float u);

  /**
   * Function to get the total attraction/repulsion velocity
   */
  float get_attraction_velocity(float u);

  void get_lattice_motion(const int &ID, const int &state_ID, float &v_x, float &v_y);
  /**
   * Implementation of method to get the commanded velocity
   */
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*CONTROLLER_CARTESIAN_H*/
