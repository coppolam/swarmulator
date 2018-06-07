#ifndef TEMPLATE_CALCULATOR_H
#define TEMPLATE_CALCULATOR_H

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "auxiliary.h"
#include "omniscient_observer.h"

using namespace std;

class Template_Calculator
{
  // Map of state-space index to possible action space indexes.
  OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.
  vector<float> blink;

public:
  Template_Calculator();
  ~Template_Calculator() {};

  std::map<int, vector<int>> state_action_matrix;
  void set_state_action_matrix(string filename);
  float get_preferred_bearing(const vector<float> &bdes, const float v_b);
  bool fill_template(vector<bool> &q, const float b_i, const float u, float dmax, float angle_err, int &j);
  void assess_situation(uint8_t ID, vector<bool> &q_old, vector<int> &q_old_ID);
};

#endif /*TEMPLATE_CALCULATOR_H*/