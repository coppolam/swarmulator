#ifndef TEMPLATE_CALCULATOR_H
#define TEMPLATE_CALCULATOR_H

#include <map>
#include <fstream>
#include <sstream>
#include <random>
#include <iterator>

#include "terminalinfo.h"
#include "omniscient_observer.h"

using namespace std;

class Template_Calculator
{
  // Map of state-space index to possible action space indexes.
  OmniscientObserver *o; // The omniscient observer is used to simulate sensing the other agents.
  vector<float> blink;

public:
  /**
   * Constructor
   */
  Template_Calculator();

  /**
   * Destructor
   */
  ~Template_Calculator() {};

  /**
   * State-action map storage
   */
  std::map<int, vector<int>> state_action_matrix;

  /**
   * Function to read the state-action map from a txt file and store it in the object
   */
  void set_state_action_matrix(string filename);

  /**
   * Function to select the desired relative bearing based on what is easiest
   */
  float get_preferred_bearing(const vector<float> &bdes, const float v_b);

  /**
   * Construct the local discrete state in binary form (q) from the continous neighborhood
   */
  bool  fill_template(vector<bool> &q, const float &b_i, const float &u, const float &dmax, const float &angle_err);

  /**
   * Define the current neighborhood
   */  
  void assess_situation(uint8_t ID, vector<bool> &q, vector<int> &q_ID);
};

#endif /*TEMPLATE_CALCULATOR_H*/