#ifndef CONTROLLER_BEARING_SHAPE_H
#define CONTROLLER_BEARING_SHAPE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

#include <map>
#include <fstream>
#include "terminalinfo.h"
#include <sstream>
#include <random>
#include <iterator>

template <typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator &g)
{
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(g));
  return start;
}

template <typename Iter>
Iter select_randomly(Iter start, Iter end)
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

using namespace std;

class Controller_Bearing_Shape: public Controller
{
  // Map of state-space index to possible action space indexes.
  std::map<int, vector<int>> state_action_matrix;

public:
  Controller_Bearing_Shape() : Controller(){
    state_action_matrix.clear();
    terminalinfo ti;

    vector<uint8_t> my_arr;
    ifstream state_action_matrix_file("./conf/state_action_matrix_file.txt");

    if (state_action_matrix_file.is_open()) {
      ti.info_msg("Opened state action matrix file.");

      // Collect the data inside the a stream, do this line by line
      while (!state_action_matrix_file.eof()) {
        string line;
        getline(state_action_matrix_file, line);
        stringstream stream_line(line);
        int buff[10];
        int columns = 0;
        int state_index ;
        bool index_checked = false;
        while (!stream_line.eof()) {
          if (!index_checked){
            stream_line >> state_index;
            index_checked = true;
          } else{
            stream_line >> buff[columns++];
          }
        }
        vector<int> actions_index(begin(buff), begin(buff)+columns);
        state_action_matrix.insert(pair<int, vector<int>>(state_index, actions_index));
      }
      state_action_matrix_file.close();
    }
    else {
      ti.debug_msg("Unable to open state action matrix file.");
    }

    std::map<int, vector<int>>::iterator it;
    int ff = 31;
    it = state_action_matrix.find(ff);
    if (it != state_action_matrix.end()){
      int r = *select_randomly(state_action_matrix.find(ff)->second.begin(), state_action_matrix.find(ff)->second.end());
      cout << r << endl;
    }

  };

  float f_attraction(float u, float b);
  float f_repulsion(float u);
  float f_extra(float u);
  float get_attraction_velocity(float u, float b_eq);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);

  float get_preferred_bearing(const vector<float> &bdes, const float v_b);
  void fill_template(vector<bool> &q, const float b_i, const float u, float dmax);
  void assess_situation(uint8_t ID, vector<bool> &q_old);

};

#endif /*CONTROLLER_BEARING_SHAPE_H*/