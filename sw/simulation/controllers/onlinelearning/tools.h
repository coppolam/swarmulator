#ifndef TOOLS_H
#define TOOLS_H
#include <armadillo>

static arma::mat pagerank(const arma::mat &G)
{
  // Define parameters
  uint maxiter = 1000;
  float tol = 0.00001;

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

#endif /*TOOLS*/