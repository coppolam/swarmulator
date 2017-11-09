#include "controller_bearing.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "auxiliary.h"

#define _ddes 0.6 // Desired equilibrium distance
#define _kr 0.1   // Repulsion gain
#define _ka 5   // Attraction gain

// The omniscient observer is used to simulate sensing the other agents.
OmniscientObserver *o = new OmniscientObserver();

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

float Controller_Bearing::f_repulsion(float u) {return -_kr / u;}
float Controller_Bearing::f_extra(float u) {return 0;}

float Controller_Bearing::get_attraction_velocity(float u, float b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u) + f_extra(u);;
}

void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x = v_r * cos(v_b);
  v_y = v_r * sin(v_b);
}

void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);

  // Additional force for for reciprocal alignment
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}


void Controller_Bearing::fill_template(vector<float> &q, const float b_i, const float u, float dmax)
{
  vector<float> blink;

  // Angles to check for neighboring links
  blink.push_back(0);
  blink.push_back(M_PI / 4.0);
  blink.push_back(M_PI / 2.0);
  blink.push_back(3 * M_PI / 4.0);
  blink.push_back(M_PI);
  blink.push_back(deg2rad(180 + 45));
  blink.push_back(deg2rad(180 + 90));
  blink.push_back(deg2rad(180 + 135));
  blink.push_back(2 * M_PI);

  // Determine link
  if (u < dmax) {
    for (int j = 0; j < (int)blink.size(); j++) {
      if (abs(b_i - blink[j]) < deg2rad(22.49)) {
        if (j == (int)blink.size() - 1) { // last element is back to 0
          q[0] = 1;
        } else {
          q[j] = 1;
        }
      }
    }
  }
}

float Controller_Bearing::get_preferred_bearing(const vector<float> &bdes, const float v_b)
{
  // Define in bv all equilibrium angles at which the agents can organize themselves
  vector<float> bv;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < (int)bdes.size(); j++) {
      bv.push_back(bdes[j]);
    }
  }

  // Find what the desired angle is in bdes
  for (int i = 0; i < (int)bv.size(); i++) {
    if (i < (int)bdes.size() * 1) {
      bv[i] = abs(bv[i] - 2 * M_PI - v_b);
    } else if (i < (int)bdes.size() * 2) {
      bv[i] = abs(bv[i] - M_PI - v_b);
    } else if (i < (int)bdes.size() * 3) {
      bv[i] = abs(bv[i] - v_b);
    } else if (i < (int)bdes.size() * 4) {
      bv[i] = abs(bv[i] + M_PI - v_b);
    } else if (i < (int)bdes.size() * 5) {
      bv[i] = abs(bv[i] + 2 * M_PI - v_b);
    }
  }

  int minindex = 0;
  for (int i = 1; i < (int)bv.size(); i++) {
    if (bv[i] < bv[minindex]) {
      minindex = i;
    }
  }

  // Reduce the index for the angle of interest from bdes
  while (minindex >= (int)bdes.size()) {
    minindex -= (int)bdes.size();
  }

  // Returned the desired equilibrium bearing
  return bdes[minindex];
}

void Controller_Bearing::assess_situation(int ID, vector<float> &q_old)
{
  vector<float> q(8, 0);                        // Set up new q template
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // Fill the template for all agents
  for (int i = 0; i < nagents - 1; i++) {
    fill_template(q,                                               // Vector to fill
                  wrapTo2Pi_f(o->request_bearing(ID, closest[i])), // Bearing
                  o->request_distance(ID, closest[i]),             // Distance
                  sqrt(pow(_ddes * 1.2, 2) + pow(_ddes * 1.2, 2)));
  }

  // Add the result to the previous template
  std::transform(q.begin(), q.end(), q_old.begin(), q_old.begin(), std::plus<float>()); // sum

  // Set to 0 any value that is not observable anymore
  for (int i = 0; i < 8; i++) {
    if (q[i] == 0) {
      q_old[i] = 0;
    }
  }
}

void Controller_Bearing::get_velocity_command(const int ID, float &v_x, float &v_y)
{
  // vector<float> situation;
  // assess_situation(ID, situation);

  v_x = 0;
  v_y = 0;
  float v_adj = 0.1;

  // Desired angles, so as to create a matrix
  vector<float> bdes;
  bdes.push_back(deg2rad(0));
  // bdes.push_back(deg2rad(  45));
  bdes.push_back(deg2rad(90));
  // bdes.push_back(deg2rad(  135));

  // Which neighbors can you sense within the range?
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // What commands does this give?
  float v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
  float b_eq = get_preferred_bearing(bdes, v_b);
  float v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);

  latticemotion(v_r, v_adj, v_b, b_eq, v_x, v_y);
}