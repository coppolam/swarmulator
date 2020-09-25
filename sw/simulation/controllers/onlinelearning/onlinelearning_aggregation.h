#ifndef ONLINELEARNING_AGGREGATION_H
#define ONLINELEARNING_AGGREGATION_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "pagerank_estimator.h"
#include "randomgenerator.h"
#include "trigonometry.h"
#include <armadillo>

class onlinelearning_aggregation: public Controller
{
  pagerank_estimator p;
  std::vector<double> motion_p; // Probability of motion
  arma::vec pol;
  uint moving_timer; // Timer measuring how long a robot has been moving
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;

public:

  onlinelearning_aggregation();
  static double fitness(const arma::vec &inputs, arma::vec *grad_out, void *opt_data);
  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*ONLINELEARNING_AGGREGATION_H*/