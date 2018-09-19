#ifndef OMNISCIENT_OBSERVER_H
#define OMNISCIENT_OBSERVER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::sort
#include <mutex>

#include "graph.h"

using namespace std;

struct indexed_array {
  float values;
  int index;
};

/* Collects data from all the agents and spits out the k-nearest for each one */
class OmniscientObserver
{
public:
  OmniscientObserver() {};
  ~OmniscientObserver() {};
  vector<int> request_closest(uint8_t ID); // request IDs of closest k neighbours and for your ID
  vector<int> request_closest_inrange(uint8_t ID, float range);
  float request_distance_dim(uint8_t ID, uint8_t ID_tracked, uint8_t dim);
  float request_distance(uint8_t ID, uint8_t ID_tracked);
  float request_bearing(uint8_t ID, uint8_t ID_tracked);
  bool connected_graph_range(float range);
  float get_centroid(uint8_t dim);
  bool see_if_moving(uint8_t ID);
  bool check_happy(void);
};

#endif /*OMNISCIENT_OBSERVER_H*/