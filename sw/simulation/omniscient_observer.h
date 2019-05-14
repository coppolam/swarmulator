#ifndef OMNISCIENT_OBSERVER_H
#define OMNISCIENT_OBSERVER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::sort
#include <mutex>

#include "graph.h"
#include "randomgenerator.h"

using namespace std;

struct indexed_array {
  float values;
  int index;
};

/**
 * Collects data from all the agents and spits out the k-nearest for each one
 */
class OmniscientObserver
{
public:
  random_generator rg;

  /**
   * Constructor
   */
  OmniscientObserver() {};

  /**
   * Destructor
   */
  ~OmniscientObserver() {};

  /**
   * Get the closest
   */
  vector<int> request_closest(uint8_t ID); // request IDs of closest k neighbours and for your ID
  
  /**
   * Get the closest within a range
   */
  vector<int> request_closest_inrange(uint8_t ID, float range);

  /**
   * Get the relative distance between two agents along x or y
   */
  float request_distance_dim(uint8_t ID, uint8_t ID_tracked, uint8_t dim);

  /**
   * Get the relative distance between two agents
   */
  float request_distance(uint8_t ID, uint8_t ID_tracked);

  /**
   * Get the relative bearing between two agents
   */
  float request_bearing(uint8_t ID, uint8_t ID_tracked);

  /**
   * Checks that the graph is connected
   */
  bool connected_graph_range(float range);

  /**
   * Returns the centroid of the swarm along the dimension dim (0=North, 1=East)
   */
  float get_centroid(uint8_t dim);

  /**
   * Returns own bearing
   */
  float own_bearing(uint8_t ID);

  /**
   * This function returns true if the agent with the specified ID is declared as moving
   * It is used to simulate the fact that a robot may detect motion by another robot
   */
  bool see_if_moving(uint8_t ID);

  /**
   * This function returns whether an agent is in a desired local state
   */
  bool check_happy(void);
};

#endif /*OMNISCIENT_OBSERVER_H*/