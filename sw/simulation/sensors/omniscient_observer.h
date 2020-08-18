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

struct indexed_array {
  float values;
  int index;
};

/**
 * Collects data from all the agents and spits out the k-nearest for each one
 */
class OmniscientObserver
{
  random_generator rg;
  float rangesensor;

public:
  /**
   * Constructor
   */
  OmniscientObserver() {};

  /**
   * Destructor
   */
  ~OmniscientObserver() {};

  /**
   * Get a std::vector of the closest neighbors, in order from closest to furthest.
   *
   * @param ID The ID of the robot that is performing the request
   * @return std::vector<uint> A std::vector of the IDs of all the neighbors, from closest to furthest.
   */
  std::vector<uint> request_closest(const uint16_t &ID); // request IDs of closest k neighbours and for your ID

  /**
   * @brief Get the closest within a range
   *
   * This function is the same as as request_closest,
   * but only returns agents within a given range, instead of the whole swarm
   *
   * @param ID The ID of the robot that is performing the request
   * @param range The maximum range that the agent can sense
   * @return std::vector<uint> A std::vector of the IDs of all the neighbors within the range, from closest to furthest.
   */
  std::vector<uint> request_closest_inrange(const uint16_t &ID, const float &range);

  /**
   * Get the relative distance between two agents along x or y
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   * @param dim The dimension along which we measure the distance (x or y)
   * @return The distance from robot ID to robot ID_tracked along the dimension dim
   */
  float request_distance_dim(const uint16_t &ID, const uint16_t &ID_tracked, const uint16_t &dim);

  /**
   * Get the relative distance between two agents
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   * @return float The distance from robot ID to robot ID_tracked
   */
  float request_distance(const uint16_t &ID, const uint16_t &ID_tracked);

  /**
   * Get the relative bearing between two agents
   *
   * @param ID The ID of the robot that is performing the request
   * @param ID_tracked The ID of the robot to be sensed
   * @return float The angle from robot ID to robot ID_tracked in radians
   */
  float request_bearing(const uint16_t &ID, const uint16_t &ID_tracked);

  /**
   * Checks that the graph is connected
   *
   * @param range range of the relative sensing between robots
   * @return bool Returns true if the swarm is connected, considering that the robots have relative sensing with a given range.
   */
  bool connected_graph_range(const float &range);

  /**
   * Returns the centroid of the swarm along the dimension dim (0=East, 1=North)
   *
   * @param dim Dimension of interest
   * @return float The position of the centroid along the dimension dim
   */
  float get_centroid(const uint16_t &dim);

  /**
   * Gets the current orientation of the robot
   *
   * @param ID The ID of the robot
   * @return float The orientation of the robot (in radians)
   */
  float own_bearing(const uint16_t &ID);

  /**
   * @brief This function returns true if the agent with the specified ID is declared as moving
   * It is used to simulate the fact that a robot may detect motion by another robot
   *
   * @param ID Gets a local state of the robot that declares whether it is active or not
   * @return bool Returns true if the robot is moving.
   */
  bool see_if_moving(const uint16_t &ID);

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
   *
   * @return bool True if happy, false if not happy
   */
  bool check_happy(void);

  /**
   * @brief Get the relative location of neighbor if they are within range
   *
   * @param ID Robot who calls the function (with the sensor)
   * @param range Extent of range sensor
   * @param r List or ranges to neighbors within the range sensor (output)
   * @param b Bearing to neighbors (output)
   */
  void relative_location_inrange(const uint16_t ID, const float range, std::vector<float> &r, std::vector<float> &b);

  /**
   * @brief Relative location to all robots
   *
   * @param ID
   * @param r
   * @param b
   */
  void relative_location(const uint16_t ID, std::vector<float> &r, std::vector<float> &b);

  /**
   * @brief Returns true if there is nearby food
   *
   * @param ID
   * @param r
   * @param b
   */
  bool sense_food(const uint16_t ID, uint16_t &food_ID, float rangesensor);

  /**
   * @brief Return the location to the beacon location in the environment
   *
   * @param ID
   * @param r
   * @param b
   */
  void beacon(const uint16_t ID, float &r, float &b);

  /**
   * @brief Set the sensor range object
   *
   * @param r
   */
  void set_sensor_range(float r)
  {
    rangesensor = r;
  }
};

#endif /*OMNISCIENT_OBSERVER_H*/
