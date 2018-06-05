#include "controller_bearing.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "auxiliary.h"

Controller_Bearing::Controller_Bearing() : Controller() {};
Controller_Bearing::~Controller_Bearing() {};

float Controller_Bearing::f_attraction(float u, float b_eq)
{
  float w;
  if (b_eq == M_PI / 2 || b_eq == 0) {
    w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
    return 1 / (1 + exp(-_ka * (u - w))); //% sigmoid function -- long-range
  } else {
    float ddes2 = sqrt(pow(_ddes, 2) + pow(_ddes, 2));
    w = log((ddes2 / _kr - 1) / exp(-_ka * ddes2)) / _ka;
    return 1 / (1 + exp(-_ka * (u - w))); //% sigmoid function -- long-range
  }
}

float Controller_Bearing::get_attraction_velocity(float u, float b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u);
}

void Controller_Bearing::attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x = v_r * cos(v_b);
  v_y = v_r * sin(v_b);
}

void Controller_Bearing::latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);

  // Additional force for for reciprocal alignment
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}

void Controller_Bearing::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  vector<bool> state(8, 0);
  vector<int> state_ID;
  // The ID is just used for simulation purposes
  t->assess_situation(ID, state, state_ID);

  v_x = 0;
  v_y = 0;

  // Desired angles, so as to create a matrix
  vector<float> bdes;
  bdes.push_back(deg2rad(0));
  // bdes.push_back(deg2rad(  45));
  bdes.push_back(deg2rad(90));
  // bdes.push_back(deg2rad(  135));

  // Which neighbors can you sense within the range?
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // What commands does this give?
  float v_b  = wrapToPi_f(o->request_bearing(ID, closest[0]));
  float b_eq = t->get_preferred_bearing(bdes, v_b);
  float v_r  = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);

  latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);

}