#ifndef ONLINELEARNING_PFSM_MOD_H
#define ONLINELEARNING_PFSM_MOD_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "pagerank_estimator.h"
#include "randomgenerator.h"
#include "trigonometry.h"
#include "template_calculator.h"
#include <armadillo>
#include "txtwrite.h"

#define COMMAND_LOCAL 1  // Local frame

class onlinelearning_pfsm_mod: public Controller
{
  Template_Calculator t;
  uint moving_timer;
  int selected_action;
  uint timelim;
  float vmean;
  arma::mat policy;
  uint st; // current state
  float vx_ref, vy_ref;
  pagerank_estimator p;
  bool done;
  uint states, actions;

#ifdef LOG
  txtwrite writer;
  std::string filename;
  std::ofstream logfile;
#endif

public:
  /**
   * @brief Construct a new onlinelearning pfsm object
   *
   */
  onlinelearning_pfsm_mod();

  /**
   * @brief Run the optimization routine and update reference to policy
   *
   * @param p Model data
   * @param policy Policy matrix (reference)
   * @param f Flag to indicate if the routine is finished
   */
  static void optimization_routine_ref(pagerank_estimator p,
                                       arma::mat &policy,
                                       bool &f);

  /**
   * @brief Run the optimization routine and return the policy
   *
   * @param p Model data
   * @param policy Policy matrix
   */
  static arma::mat optimization_routine(pagerank_estimator p,
                                        arma::mat policy);

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

  void action_motion(const int &selected_action,
                     float r, float t, float &v_x, float &v_y);

  void state_action_lookup(const int ID, uint state_index);

  virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);

  virtual void animation(const uint16_t ID);
};

#endif /*ONLINELEARNING_PFSM_MOD_H*/