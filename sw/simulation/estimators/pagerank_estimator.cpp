#include "pagerank_estimator.h"
#include "main.h"
#include "fmat.h"
#include <fstream>
#include <iostream>
#include "fitness_functions.h"

#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

pagerank_estimator::pagerank_estimator(uint s_size, uint n_actions)
{
  estimator_active = false;
  if (s_size > 0 && n_actions > 0) {
    estimator_active = true;
    n_states = s_size;
    H.zeros(n_states, n_states);
    E.zeros(n_states, n_states);
    for (size_t i = 0; i < n_actions; i++) {
      A.push_back(H);
    }
  }
};

void pagerank_estimator::init(bool all)
{
  int n;
  if (all) {
    n = nagents;
  } else {
    n = 1;
  }

  if (estimator_active) {
    s_k.assign(n, 0);
    s_kp1.assign(n, 0);
    fitness = 0;
  }
}

void pagerank_estimator::extend(void)
{
  if (estimator_active) {
    if (s.size() > nagents) {
      s_k.resize(s.size(), 0);
      s_kp1.resize(s.size(), 0);
      s_k.assign(s.size(), 0);
      s_kp1.assign(s.size(), 0);
    }
  }
}

void pagerank_estimator::update(const uint &ID, const int &st, const uint &a)
{
  if (estimator_active) {
    s_kp1[ID] = st; // new state
    update_G(ID, a); // update G = f(H,E,A) estimate
    s_k[ID] = st; // update state
  }
}

void pagerank_estimator::update_G(const uint &ID, const uint &action)
{
  if (estimator_active) {
    // Update H or E depending on whether the change was your own
    if (action > 0) {
      H(s_kp1[ID], s_k[ID]) += 1;
      A[action - 1](s_kp1[ID], s_k[ID]) += 1;
    } else {
      E(s_kp1[ID], s_k[ID]) += 1;
    }
  }
}

void pagerank_estimator::print(void)
{
  if (estimator_active) {
    cout << "H" << endl;
    H.print();
    cout << "E" << endl;
    E.print();
    for (size_t i = 0; i < A.size(); i++) {
      cout << "A" << i << endl;
      A[i].print();
    }
  }
}

void pagerank_estimator::save(void)
{
  if (estimator_active) {
    H.save("logs/H_" + identifier + ".csv", arma::csv_ascii);
    E.save("logs/E_" + identifier + ".csv", arma::csv_ascii);
    for (size_t i = 0; i < A.size(); i++) {
      A[i].save("logs/A_" + identifier + "_" + to_string(i) + ".csv", arma::csv_ascii);
    }
  }
}