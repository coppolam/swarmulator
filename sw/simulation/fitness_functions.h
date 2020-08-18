#ifndef FITNESS_FUNCTIONS_H
#define FITNESS_FUNCTIONS_H

#include "omniscient_observer.h"

/**
 * Mean number of neighbors
 *
 * @return float the mean number of neighbors that each robot has
 */
inline static float mean_number_of_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    std::vector<uint> closest = o.request_closest_inrange(ID, s[ID]->controller->get_max_sensor_range());
    f += (float)closest.size() / (float)s.size();
  }
  return f;
}

/**
 * Mean distance to all
 *
 * @return float the mean distance between all robots
 */
inline static float mean_dist_to_all()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    std::vector<float> r, b;
    o.relative_location(ID, r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)s.size();
  }
  return f;
}

/**
 * Mean distance to neighbors
 *
 * @return float the mean distance between neighboring robots
 */
inline static float mean_dist_to_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    std::vector<float> r, b;
    o.relative_location_inrange(ID, s[ID]->controller->get_max_sensor_range(), r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)s.size();
  }
  return f;
}

/**
 * Connectivity
 *
 * Change f to 0.0 if the graph of the swarm is disconnected, else keeps f
 */
inline static void connectivity_check(float &f)
{
  OmniscientObserver o;
  if (!(o.connected_graph_range(s[0]->controller->get_max_sensor_range()))) {
    f = 0.0;
  }
}

/**
 * @brief Measures the number of clusters that are formed
 *
 * @return uint
 */
inline static uint number_of_clusters()
{
  OmniscientObserver o;
  Graph g(s.size());
  for (size_t ID = 0; ID < s.size(); ID++) {
    std::vector<uint> neighbors = o.request_closest_inrange(ID, s[ID]->controller->get_max_sensor_range());
    for (size_t j = 0; j < neighbors.size(); j++) {
      g.addEdge(ID, neighbors[j]);
    }
  }
  uint a = g.connectedComponents();
  return a;
}

/**
 * Select a fitness function, or use your own if you want.
 *
 * @return float fitness
 */
inline static float evaluate_fitness()
{
  float f = 1.; // Default

  // Define the fitness function that you would like to use, or write your own
  if (!strcmp(param->fitness().c_str(), "mean_number_of_neighbors"))
  { f = mean_number_of_neighbors();}
  else if (!strcmp(param->fitness().c_str(), "mean_dist_to_neighbors"))
  { f = mean_dist_to_neighbors();}
  else if (!strcmp(param->fitness().c_str(), "aggregation_clusters"))
  { f = 1. / ((float)number_of_clusters() / float(nagents));}
  else if (!strcmp(param->fitness().c_str(), "dispersion_clusters"))
  { f = ((float)number_of_clusters() / float(nagents));}
  else if (!strcmp(param->fitness().c_str(), "aggregation_dist_to_all"))
  { f = 1. / mean_dist_to_all();}
  else if (!strcmp(param->fitness().c_str(), "dispersion_dist_to_all"))
  { f = mean_dist_to_all();}
  else if (!strcmp(param->fitness().c_str(), "food"))
  { f = (float)environment.nest;}
  else if (!strcmp(param->fitness().c_str(), "connected"))
  { connectivity_check(f);}
  return f;
}

#endif
