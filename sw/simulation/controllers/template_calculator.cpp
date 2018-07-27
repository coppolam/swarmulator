#include "template_calculator.h"
#include "main.h"
#include "auxiliary.h"
#include <algorithm> // std::find

Template_Calculator::Template_Calculator()
{
  // Angles to check for neighboring links
  blink.push_back(0);
  blink.push_back(M_PI / 4.0);
  blink.push_back(M_PI / 2.0);
  blink.push_back(3 * M_PI / 4.0);
  blink.push_back(M_PI);
  blink.push_back(deg2rad(180 + 45));
  blink.push_back(deg2rad(180 + 90));
  blink.push_back(deg2rad(180 + 135));
  blink.push_back(2 * M_PI);
};

void Template_Calculator::set_state_action_matrix(string filename)
{
  state_action_matrix.clear();
  terminalinfo ti;
  ifstream state_action_matrix_file(filename);

  if (state_action_matrix_file.is_open()) {
    ti.info_msg("Opened state action matrix file.");

    // Collect the data inside the a stream, do this line by line
    while (!state_action_matrix_file.eof()) {
      string line;
      getline(state_action_matrix_file, line);
      stringstream stream_line(line);
      int buff[10] = {};
      int columns = 0;
      int state_index;
      bool index_checked = false;
      while (!stream_line.eof()) {
        if (!index_checked) {
          stream_line >> state_index;
          index_checked = true;
        } else {
          stream_line >> buff[columns];
          columns++;
        }
      }
      if (columns > 0) {
        vector<int> actions_index(begin(buff), begin(buff) + columns);
        state_action_matrix.insert(pair<int, vector<int>>(state_index, actions_index));
      }
    }
    state_action_matrix_file.close();
  } else {
    ti.debug_msg("Unable to open state action matrix file.");
  }
}

// TODO: Make smarter
bool Template_Calculator::fill_template(vector<bool> &q, const float &b_i, const float &u, const float &dmax, const float &angle_err)
{
  // Determine link (cycle through all options)
  if (u < dmax) { // If in range of sensor
    for (int j = 0; j < (int)blink.size(); j++) { // For all angle options
      if (abs(b_i - blink[j]) < deg2rad(angle_err) && !q[j]) {   // If in the right angle and not already taken by another agent
        if (j == (int)blink.size() - 1) { // last element is back to 0
          j = 0;
        }
        q[j] = true;
        return true;
      }
    }
  }
  return false;
}

// TODO: Make smarter
float Template_Calculator::get_preferred_bearing(const vector<float> &bdes, const float v_b)
{
  // Define in bv all equilibrium angles at which the agents can organize themselves
  vector<float> bv;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < (int)bdes.size(); j++) {
      bv.push_back(bdes[j]);
    }
  }

  // Find what the desired angle is in bdes
  for (int i = 0; i < (int)bv.size(); i++) {
    if (i < (int)bdes.size() * 1) {
      bv[i] = abs(bv[i] - 2 * M_PI - v_b);
    } else if (i < (int)bdes.size() * 2) {
      bv[i] = abs(bv[i] - M_PI - v_b);
    } else if (i < (int)bdes.size() * 3) {
      bv[i] = abs(bv[i] - v_b);
    } else if (i < (int)bdes.size() * 4) {
      bv[i] = abs(bv[i] + M_PI - v_b);
    } else if (i < (int)bdes.size() * 5) {
      bv[i] = abs(bv[i] + 2 * M_PI - v_b);
    }
  }

  int minindex = 0;
  for (int i = 1; i < (int)bv.size(); i++) {
    if (bv[i] < bv[minindex]) {
      minindex = i;
    }
  }

  // Reduce the index for the angle of interest from bdes
  while (minindex >= (int)bdes.size()) {
    minindex -= (int)bdes.size();
  }

  // Returned the desired equilibrium bearing
  return bdes[minindex];
}

void Template_Calculator::assess_situation(uint8_t ID, vector<bool> &q, vector<int> &q_ID)
{
  q.clear();
  q.assign(8, false);
  q_ID.clear();

  // Fill the template with respect to the agent in question
  vector<int> closest = o->request_closest(ID);
  for (uint8_t i = 0; i < nagents - 1; i++) {
    if (fill_template(q, // Vector to fill
                      wrapTo2Pi_f(o->request_bearing(ID, closest[i])), // Bearing
                      o->request_distance(ID, closest[i]), // Distance
                      rangesensor, 22.499)) { // Sensor range, bearing precision
      q_ID.push_back(closest[i]); // Log ID (for simulation purposes only, depending on assumptions)
    }
  }
}