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
   * Get a vector of the closest neighbors, in order from closest to furthest.
   *
   * @param ID The ID of the robot that is performing the request
   *
   * @return A vector of the IDs of all the neighbors, from closest to furthest.
   */
  vector<uint> request_closest(uint8_t ID); // request IDs of closest k neighbours and for your ID

  /**
   * @brief Get the closest within a range
   *
   * This function is the same as as request_closest,
   * but only returns agents within a given range, instead of the whole swarm
   *
   * @param ID The ID of the robot that is performing the request
   * @param range The maximum range that the agent can sense
   *
   * @return A vector of the IDs of all the neighbors within the range, from closest to furthest.
   */
  vector<uint> request_closest_inrange(uint8_t ID, float range);

  /**
   * Get the relative distance between two agents along x or y
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   * @param dim The dimension along which we measure the distance (x or y)
   *
   * @return The distance from robot ID to robot ID_tracked along the dimension dim
   */
  float request_distance_dim(uint8_t ID, uint8_t ID_tracked, uint8_t dim);

  /**
   * Get the relative distance between two agents
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   *
   * @return The distance from robot ID to robot ID_tracked
   */
  float request_distance(uint8_t ID, uint8_t ID_tracked);

  /**
   * Get the relative bearing between two agents
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   *
   * @return The angle from robot ID to robot ID_tracked in radians
   */
  float request_bearing(uint8_t ID, uint8_t ID_tracked);

  /**
   * Checks that the graph is connected
   *
   * @param range range of the relative sensing between robots
   *
   * @return Returns true if the swarm is connected, considering that the robots have relative sensing with a given range.
   */
  bool connected_graph_range(float range);

  /**
   * Returns the centroid of the swarm along the dimension dim (0=East, 1=North)
   *
   * @param dim Dimension of interest
   *
   * @return The position of the centroid along the dimension dim
   */
  float get_centroid(uint8_t dim);

  /**
   * Gets the current orientation of the robot
   *
   * @param ID The ID of the robot
   *
   * @return The orientation of the robot (in radians)
   */
  float own_bearing(uint8_t ID);

  /**
   * @brief This function returns true if the agent with the specified ID is declared as moving
   * It is used to simulate the fact that a robot may detect motion by another robot
   *
   * @param ID Gets a local state of the robot that declares whether it is active or not
   *
   * @return Returns true if the robot is moving.
   */
  bool see_if_moving(uint8_t ID);

  /**
   * @brief This function returns whether all the agents in the swarm are in a desired local state
   *
   * The idea of desired local states is treate in these two papers:
   *  - Provable self-organizing pattern formation by a swarm of robots with limited knowledge
   *    Mario Coppola, Jian Guo, Eberhard Gill & Guido C. H. E. de Croon. Swarm Intelligence. Volume 13, Pages 59–94. (2019)
   *    https://link.springer.com/article/10.1007/s11721-019-00163-0
   *  - The PageRank algorithm as a method to optimize swarm behavior through local analysis
   *    M. Coppola, J. Guo, E. Gill & G. C. H. E. de Croon. Swarm Intelligence, Volume 13, Pages 277–319. (2019)
   *    https://link.springer.com/article/10.1007/s11721-019-00172-z
   */
  bool check_happy(void);

  void request_relative_location_inrange(uint8_t ID, float range, vector<float> &r, vector<float> &b);
};

#endif /*OMNISCIENT_OBSERVER_H*/