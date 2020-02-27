#ifndef FITNESS_FUNCTIONS_H
#define FITNESS_FUNCTIONS_H

#include "omniscient_observer.h"

/**
 * Mean number of neighbors
 *
 * @return float the mean number of neighbors that each robot has
 */
float mean_number_of_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < nagents; ID++) {
    vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
    f += (float)closest.size() / (float)nagents;
  }
  return f;
}

/**
 * Mean distance to neighbors
 *
 * @return float the mean distance between all neighbors
 */
float mean_dist_to_neighbors()
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < nagents; ID++) {
    vector<float> r, b;
    o.request_relative_location_inrange(ID, rangesensor, r, b);
    float r_mean = accumulate(r.begin(), r.end(), 0.0) / r.size();
    f += (float)r_mean / (float)nagents;
  }
  return f;
}

/**
 * Mean distance to neighbors
 *
 * @return float the mean distance between all neighbors
 */
float mean_dist_to_one_neighbor(uint8_t ID_tracked)
{
  float f = 0.;
  OmniscientObserver o;
  for (size_t ID = 0; ID < nagents; ID++) {
    f += o.request_distance(ID,ID_tracked)/(float)nagents;
  }
  return 1/f;
}

/**
 * Connectivity
 *
 * @return float returns 0.0 if the graph of the swarm is disconnected, else keeps f
 */
void connectivity_check(float &f)
{
  OmniscientObserver o;
  if (!(o.connected_graph_range(rangesensor))) {
    f = 0.0;
  }
}

#endif