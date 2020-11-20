#ifndef ONLINELEARNING_FORAGE_H
#define ONLINELEARNING_FORAGE_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "pagerank_estimator.h"
#include "randomgenerator.h"
#include "trigonometry.h"
#include <armadillo>
#include "txtwrite.h"

#define COMMAND_LOCAL 1  // Local frame

class onlinelearning_forage: public Controller
{
  uint timer; // Timer measuring how long a robot has been moving
  bool explore;
  float vmean;
  float timelim;
  float v_x_ref, v_y_ref;
  uint st;
  bool holds_food, choose;
  float state;

  pagerank_estimator p;
  arma::vec policy;
  bool done;
  uint states, actions;

#ifdef LOG
  txtwrite writer;
  std::string filename;
  std::ofstream logfile;
#endif

public:

  onlinelearning_forage();

  /**
   * @brief Run the optimization routine and update reference to policy
   *
   * @param p Model data
   * @param policy Policy matrix (reference)
   * @param f Flag to indicate if the routine is finished
   */
  static void optimization_routine_ref(pagerank_estimator p,
                                       arma::vec &policy,
                                       bool &f);

  /**
   * @brief Run the optimization routine and return the policy
   *
   * @param p Model data
   * @param policy Policy matrix
   */
  static arma::vec optimization_routine(pagerank_estimator p,
                                        arma::vec policy);

  /**
   * @brief PageRank fitness function
   *
   * @param inputs Input vector to optimize
   * @param data
   * @return double Fitness
   */
  static double fitness(const arma::vec &inputs,
                        arma::vec *grad_out,
                        void *opt_data);

  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
  virtual void animation(const uint16_t ID);
};

#endif /*ONLINELEARNING_FORAGE_H*/