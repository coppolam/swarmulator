/*
* C++ Program to Check whether Undirected Graph is Connected using BFS
* Code extracted from Sanfoundry Global Education & Learning Series
* http://www.sanfoundry.com/cpp-program-check-undirected-graph-connected-bfs/
*/
#ifndef RANDOM_GENERATOR_H
#define RANDOM_GENERATOR_H

#include "stdlib.h"
#include "math.h"
#include <vector>
#include <random>

using namespace std;

/**
 * Defines a graph structure that can be used to assess the topology of the swarm
 */
class random_generator
{
  default_random_engine generator;

public:

  /**
   * Constructor instatiates a list of all connections in the graph.
   */
  random_generator();

  /**
   * Get a random value of type float between a min and max
   */
  float uniform_float(float min, float max);

  /**
   * Get a random value of type int between a min and a max
   */
  int uniform_int(int min, int max);

  /**
   * Get a random value of type int between a min and a max
   */
  float gaussian_float(float mean, float stddev);

  /**
   * Get a random bool value with probability p
   */
  bool bernoulli(float p);

  /**
   * Generate a random vector with zero mean from a gaussian distribution
   */
  vector<float> gaussian_float_vector(const int &length, const float &mean, const float &std);

  /**
   * Generate a random vector with zero mean from a uniform distribution
   */
  vector<float> uniform_float_vector(const int &length, const float &min, const float &max);
};


#endif /* GRAPH_H */

