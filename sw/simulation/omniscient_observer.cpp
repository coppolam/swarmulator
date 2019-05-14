#include "omniscient_observer.h"
#include "main.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

typedef Matrix<bool, Dynamic, Dynamic> MatrixXb;

int compare_index(const void *p1, const void *p2)
{
  indexed_array *elem1 = (indexed_array *)p1;
  indexed_array *elem2 = (indexed_array *)p2;

  if (elem1->values < elem2->values) {
    return -1;
  } else if (elem1->values > elem2->values) {
    return 1;
  } else {
    return 0;
  }
}

void array_sortmintomax_index(int length, indexed_array *x)
{
  qsort(x, length, sizeof(struct indexed_array), compare_index);
  return;
}

vector<int> OmniscientObserver::request_closest(uint8_t ID)
{
  indexed_array dm[nagents - 1];
  vector<int> ind;
  for (uint8_t i = 0; i < nagents; i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(nagents, dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint8_t i = 1; i < nagents; i++) {
    ind.push_back(dm[i].index);
  }

  return ind;
}

vector<int> OmniscientObserver::request_closest_inrange(uint8_t ID, float range)
{
  indexed_array dm[nagents - 1];
  vector<int> ind;
  for (uint8_t i = 0; i < nagents; i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(nagents, dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint8_t i = 1; i < nagents; i++) {
    if (dm[i].values < range) {
      ind.push_back(dm[i].index);
    }
  }
  return ind;
}

bool OmniscientObserver::check_happy()
{
  bool happy = true;
  for (uint8_t ID = 0; ID < nagents; ID++) {
    if (!s[ID]->happy) {
      return false;
    }
  }

  return happy;
}

bool OmniscientObserver::connected_graph_range(float range)
{
  Graph g(nagents);

  for (uint8_t i = 0; i < nagents; i++) {
    for (uint8_t j = 0; j < nagents; j++) {
      if (request_distance(i, j) < range) {
        g.addEdge(i, j);
      }
    }
  }

  if (g.isConnected()) {
    return true;
  }
  return false;
}

float OmniscientObserver::get_centroid(uint8_t dim)
{
  float c = 0;
  for (uint8_t i = 0; i < nagents; i++) {
    c += s[i]->get_position(dim) / (float)nagents;
  }
  return c;
}

float OmniscientObserver::request_distance_dim(uint8_t ID, uint8_t ID_tracked, uint8_t dim)
{
  return s[ID_tracked]->get_position(dim) - s[ID]->get_position(dim);
}

float OmniscientObserver::request_distance(uint8_t ID, uint8_t ID_tracked)
{
  float u = 0;
  for (uint8_t i = 0; i < 2; i++) {
    float dd = s[ID_tracked]->get_position(i) - s[ID]->get_position(i);
    u += pow(dd, 2);
  }
  float noise = rg.gaussian_float(0.0, NOISE_R);
  return sqrt(u) + noise;
}

float OmniscientObserver::own_bearing(uint8_t ID)
{
  return s[ID]->get_orientation();
}

float OmniscientObserver::request_bearing(uint8_t ID, uint8_t ID_tracked)
{
  float noise = rg.gaussian_float(0.0, NOISE_B);
  float b = atan2(request_distance_dim(ID, ID_tracked, 1), request_distance_dim(ID, ID_tracked, 0)) + noise;
  #if COMMAND_LOCAL
      return b - own_bearing(ID);
  #else
    return b;
  #endif

}

bool OmniscientObserver::see_if_moving(uint8_t ID)
{
  return s[ID]->moving;
}
