#ifndef FITNESS_FUNCTIONS_H
#define FITNESS_FUNCTIONS_H

#include "omniscient_observer.h"
#include "main.h"

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
    std::vector<uint> closest = o.request_closest_inrange(ID, rangesensor);
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
    o.relative_location_inrange(ID, rangesensor, r, b);
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
  if (!(o.connected_graph_range(rangesensor))) {
    f = 0.0;
  }
}

/**
 * Source distance
 *
 * Gives a better fitness for smaller distance to source
 */
inline static void source_distance(float &f)
{
  float source_dist = 0;
  for(uint ID =0; ID<nagents; ID++)
  {
    source_dist += std::sqrt(pow((s.at(ID)->state[1]-environment.gas_obj.source_location[0]-environment.x_min),2)+pow((s.at(ID)->state[0]-environment.gas_obj.source_location[1]-environment.y_min),2));
  }
  f = source_dist/nagents;

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
    std::vector<uint> neighbors = o.request_closest_inrange(ID, rangesensor);
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
  else if (!strcmp(param->fitness().c_str(), "source_distance"))
  { source_distance(f);}
  return f;
}

#endif
