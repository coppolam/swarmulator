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
    H.assign(pow(n_states, 2), 0);
    E.assign(pow(n_states, 2), 0);
    for (size_t i = 0; i < n_actions; i++) {
      A.push_back(H);
    }
  }
};

void pagerank_estimator::init(bool all)
{
  int n;
  if (!all) {
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
    s_kp1[ID] = st; // update state
    update_G(ID, a); // update G = f(H,E,A) estimate
    s_k[ID] = st; // update state
  }
}

void pagerank_estimator::update_G(const uint &ID, const uint &action)
{
  if (estimator_active) {
    // Update H or E depending on whether the change was your own
    if (action > 0) {
      H[s_kp1[ID] + s_k[ID] * n_states] += 1; // col + row * row_length
      A[action - 1][s_kp1[ID] + s_k[ID] * n_states] += 1;
    } else {
      E[s_kp1[ID] + s_k[ID] * n_states] += 1; // col + row * row_length
    }
  }
}

void pagerank_estimator::print(void)
{
  if (estimator_active) {
    cout << "********************" << endl;
    fmat<uint>::print(n_states, n_states, H, "H");
    fmat<uint>::print(n_states, n_states, E, "E");
    for (size_t i = 0; i < A.size(); i++) {
      fmat<uint>::print(n_states, n_states, A[i], "A");
    }
  }
}

void pagerank_estimator::save(void)
{
  if (estimator_active) {
    fmat<uint>::write_to_csv("logs/E_" + identifier + ".csv", E, n_states, n_states);
    fmat<uint>::write_to_csv("logs/H_" + identifier + ".csv", H, n_states, n_states);
    for (size_t i = 0; i < A.size(); i++) {
      fmat<uint>::write_to_csv("logs/A_" + identifier + "_" + to_string(i) + ".csv", A[i], n_states, n_states);
    }
  }
}