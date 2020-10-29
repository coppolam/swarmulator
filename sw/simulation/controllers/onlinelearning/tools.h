#ifndef TOOLS_H
#define TOOLS_H
#include <armadillo>

// Enable the Armadillo library for matrix operations
// and load the optim package header
#define OPTIM_ENABLE_ARMA_WRAPPERS
#include "optim.hpp"
#include "main.h"

static arma::mat pagerank(const arma::mat &G)
{
  // Define parameters
  uint maxiter = 10000;
  float tol = 1e-8;

  // Initialize
  uint i = 0;
  float residual = 1;
  arma::mat pr = arma::normalise(arma::ones(1, G.n_rows) / G.n_rows, 1, 1);
  arma::mat pr_prev;

  // PageRank iteration routine
  while (residual >= tol && i < maxiter) {
    pr_prev = pr;
    pr = pr * G;
    residual = arma::norm(pr - pr_prev);
    i++;
  }

  // Return
  return arma::normalise(pr, 1, 1);
}

static void optimization_settings(optim::algo_settings_t &settings)
{
  settings.vals_bound = false;
  settings.lower_bounds = arma::zeros(param->pr_states() * param->pr_actions(), 1);
  settings.upper_bounds = arma::ones(param->pr_states() * param->pr_actions(), 1);
}

#endif /*TOOLS*/