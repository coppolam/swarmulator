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
    vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
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
    vector<float> r, b;
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
    vector<float> r, b;
    o.relative_location_inrange(ID, rangesensor, r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)s.size();
  }
  return f;
}

/**
 * Mean distance to neighbors
 *
 * @return float the mean distance to all neighbors that a robot has
 */
inline static float mean_dist_to_one_neighbor(uint8_t ID_tracked)
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < s.size(); ID++) {
    f += o.request_distance(ID, ID_tracked) / (float)s.size();
  }
  return 1 / f;
}

/**
 * Connectivity
 * Change f to 0.0 if the graph of the swarm is disconnected, else keeps f
 */
inline static void connectivity_check(float &f)
{
  OmniscientObserver o;
  if (!(o.connected_graph_range(rangesensor))) {
    f = 0.0;
  }
}

inline static uint number_of_clusters()
{
  OmniscientObserver o;
  Graph g(s.size());
  for (size_t ID = 0; ID < s.size(); ID++) {
    vector<uint> neighbors = o.request_closest_inrange(ID, rangesensor);
    for (size_t j = 0; j < neighbors.size(); j++) {
      g.addEdge(ID, neighbors[j]);
    }
  }
  uint a = g.connectedComponents();
  return a;
}

/**
 * Select a fitness function, or use your own if you want.
 * TODO: Move to controllers so that its defined within a particular controller. It would be far more versatile.
 * @return float fitness
 */
inline static float evaluate_fitness()
{
  float f;
  // f = mean_dist_to_one_neighbor(0);
  // f = mean_number_of_neighbors();
  f = 1.0 / (float)number_of_clusters(); // use 1./ to minimize (e.g. for aggregation)
  // f = mean_dist_to_neighbors();
  // f = 1. / mean_dist_to_all(); //  use 1./ to minimize
  // connectivity_check(f);
  return f;
}

#endif
