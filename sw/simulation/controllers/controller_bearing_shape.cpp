#include "controller_bearing_shape.h"
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include "omniscient_observer.h"
#include "auxiliary.h"

#define _ddes 1.0 // Desired equilibrium distance
#define _kr 0.1 // Repulsion gain
#define _ka 5 // Attraction gain
#define _v_adj 0.3 // Adjustment velocity

// The omniscient observer is used to simulate sensing the other agents.
OmniscientObserver *o = new OmniscientObserver();

Controller_Bearing_Shape::Controller_Bearing_Shape() : Controller()
{
  state_action_matrix.clear();
  terminalinfo ti;
  ifstream state_action_matrix_file("./conf/state_action_matrix.txt");

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
        vector<int> actions_index( begin(buff), begin(buff) + columns );
        state_action_matrix.insert( pair<int, vector<int>>(state_index, actions_index) );
      }
    }
    state_action_matrix_file.close();
  }
  else {
    ti.debug_msg("Unable to open state action matrix file.");
  }

}

float Controller_Bearing_Shape::f_attraction(float u, float b_eq)
{
  float w = log((_ddes / _kr - 1) / exp(-_ka * _ddes)) / _ka;
  return 1 / (1 + exp(-_ka * (u - w)));
}

float Controller_Bearing_Shape::f_repulsion(float u) { return -_kr / u; }
float Controller_Bearing_Shape::f_extra(float u) {return 0;}

float Controller_Bearing_Shape::get_attraction_velocity(float u, float b_eq)
{
  return f_attraction(u, b_eq) + f_repulsion(u) + f_extra(u);;
}

void attractionmotion(const float &v_r, const float &v_b, float &v_x, float &v_y)
{
  v_x = v_r * cos(v_b);
  v_y = v_r * sin(v_b);
}

void latticemotion(const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v_x, float &v_y)
{
  attractionmotion(v_r + v_adj, v_b, v_x, v_y);

  // Additional force for for reciprocal alignment
  v_x += -v_adj * cos(bdes * 2 - v_b);
  v_y += -v_adj * sin(bdes * 2 - v_b);
}


bool Controller_Bearing_Shape::fill_template(vector<bool> &q, const float b_i, const float u, float dmax)
{
  vector<float> blink;

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

  // Determine link (cycle through all options)
  if (u < dmax) {
    for (int j = 0; j < (int)blink.size(); j++) {
      if (abs(b_i - blink[j]) < deg2rad(22.49)) {
        if (j == (int)blink.size() - 1) { // last element is back to 0
          q[0] = true;
        } else {
          q[j] = true;
        }
        return true;
      }
    }
  }
  
  return false;

}

float Controller_Bearing_Shape::get_preferred_bearing(const vector<float> &bdes, const float v_b)
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

void Controller_Bearing_Shape::assess_situation(uint8_t ID, vector<bool> &q, vector<int> &q_ID)
{
  q.clear();
  q.assign(8,false);

  vector<int> closest = o->request_closest(ID); // Get vector of all neighbor IDs from closest to furthest

  // Fill the template with respect to the agent in question
  for (uint8_t i = 0; i < nagents - 1; i++) {
    if (fill_template(q,                                               // Vector to fill
                  wrapTo2Pi_f(o->request_bearing(ID, closest[i])), // Bearing
                  o->request_distance(ID, closest[i]),             // Distance
                  sqrt(pow(_ddes * 1.2, 2) + pow(_ddes * 1.2, 2)))){ // Sensor range
                    q_ID.push_back(closest[i]);
                  }
  }

}

vector<bool> moving(50, 0);
vector<int> moving_timer(50, 0);

void Controller_Bearing_Shape::get_velocity_command(const uint8_t ID, float &v_x, float &v_y)
{
  v_x = 0;
  v_y = 0;
  
  // Desired angles, so as to create a matrix
  vector<float> bdes;
  bdes.push_back(deg2rad(0));
  bdes.push_back(deg2rad(45));
  bdes.push_back(deg2rad(90));
  bdes.push_back(deg2rad(135));

  // Which neighbors can you sense within the range?
  vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest

  // What commands does this give?
  float v_b = wrapToPi_f(o->request_bearing(ID, closest[0]));
  float b_eq = get_preferred_bearing(bdes, v_b);
  float v_r = get_attraction_velocity(o->request_distance(ID, closest[0]), b_eq);

  vector<bool> q(8, 0); // Make situation vector q
  vector<int>  q_ID; // Make situation vector q
  assess_situation(ID, q, q_ID);
  int state_index = bool2int(q); // Convert q to integer value

  // Extract random action
  std::map<int, vector<int>>::iterator it;
  it = state_action_matrix.find(state_index);
  int r = -1;
  // Is there an action to take
  if (it != state_action_matrix.end()) {
    r = *select_randomly(state_action_matrix.find(state_index)->second.begin(),
                         state_action_matrix.find(state_index)->second.end()); 
  }

  // cout << (int)ID << " q = ";
  // for (uint8_t i = 0; i < 8; i++)
  //   cout << q[i] << " ";
  // cout << endl << "q_ID = ";
  // for (uint8_t i = 0; i < q_ID.size(); i++) {
  //   cout << q_ID[i] << " ";
  // }
  // cout << endl;

  // If I'm not moving yet, are my neighbors moving?
  bool canImove = true;
  for (uint8_t i = 0; i < q_ID.size(); i++) {
    if (moving[q_ID[i]]) {
      // Somebody nearby is moving
      canImove = false;
    }
  }

  cout << (int)ID <<" move?="<< (int)canImove << " action=" << r+1 << endl;
  
  // Apply action if needed
  if (( (r > -1 && canImove) || (moving[ID] && moving_timer[ID]<50) ) && moving_timer[ID] < 50) {
    int actionspace_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    int actionspace_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    v_x = _v_adj * (float)actionspace_x[r];
    v_y = _v_adj * (float)actionspace_y[r];
    moving[ID] = true;
    moving_timer[ID]++;
  } else if (!canImove) {
    moving[ID] = false;
    attractionmotion(v_r, v_b, v_x, v_y);
    moving_timer[ID] = 0;
  } else {
    moving[ID] = false;
    latticemotion(v_r, _v_adj, v_b, b_eq, v_x, v_y);
    moving_timer[ID] = 0;
  }
  keepbounded(v_x, -1, 1);
  keepbounded(v_y, -1, 1);
}