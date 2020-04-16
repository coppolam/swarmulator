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

vector<uint> OmniscientObserver::request_closest(const uint8_t &ID)
{
  indexed_array dm[s.size() - 1];
  vector<uint> ind;
  for (uint8_t i = 0; i < s.size(); i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(s.size(), dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint8_t i = 1; i < s.size(); i++) {
    ind.push_back(dm[i].index);
  }

  return ind;
}

vector<uint> OmniscientObserver::request_closest_inrange(const uint8_t &ID, const float &range)
{
  indexed_array dm[s.size() - 1];
  vector<uint> ind;
  for (uint8_t i = 0; i < s.size(); i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(s.size(), dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint8_t i = 1; i < s.size(); i++) {
    if (dm[i].values < range) {
      ind.push_back(dm[i].index);
    }
  }
  return ind;
}

bool OmniscientObserver::check_happy()
{
  bool happy = true;
  for (uint8_t ID = 0; ID < s.size(); ID++) {
    if (!s[ID]->happy) {
      return false;
    }
  }

  return happy;
}

bool OmniscientObserver::connected_graph_range(const float &range)
{
  Graph g(s.size());

  for (uint8_t i = 0; i < s.size(); i++) {
    for (uint8_t j = 0; j < s.size(); j++) {
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

float OmniscientObserver::get_centroid(const uint8_t &dim)
{
  float c = 0;
  for (uint8_t i = 0; i < s.size(); i++) {
    c += s[i]->get_position(dim) / (float)s.size();
  }
  return c;
}

float OmniscientObserver::request_distance_dim(const uint8_t &ID, const uint8_t &ID_tracked, const uint8_t &dim)
{
  return s[ID_tracked]->get_position(dim) - s[ID]->get_position(dim);
}

float OmniscientObserver::request_distance(const uint8_t &ID, const uint8_t &ID_tracked)
{
  float u = 0;
  for (uint8_t i = 0; i < 2; i++) {
    float dd = s[ID_tracked]->get_position(i) - s[ID]->get_position(i);
    u += pow(dd, 2);
  }
  float noise = rg.gaussian_float(0.0, NOISE_R);
  return sqrt(u) + noise;
}

float OmniscientObserver::own_bearing(const uint8_t &ID)
{
  return s[ID]->get_orientation();
}

float OmniscientObserver::request_bearing(const uint8_t &ID, const uint8_t &ID_tracked)
{
  float noise = rg.gaussian_float(0.0, NOISE_B);
  float b = atan2(request_distance_dim(ID, ID_tracked, 1), request_distance_dim(ID, ID_tracked, 0)) + noise;
#if COMMAND_LOCAL
  return b - own_bearing(ID);
#else
  return b;
#endif

}

bool OmniscientObserver::see_if_moving(const uint8_t &ID)
{
  return s[ID]->moving;
}

void OmniscientObserver::relative_location_inrange(const uint8_t ID, const float range, vector<float> &r,
    vector<float> &b)
{
  vector<uint> closest = request_closest_inrange(ID, range);
  for (size_t i = 0; i < closest.size(); i++) {
    r.push_back(request_distance(ID, closest[i]));
    b.push_back(request_bearing(ID, closest[i]));
  }
}

void OmniscientObserver::relative_location(const uint8_t ID, vector<float> &r, vector<float> &b)
{
  vector<uint> c = request_closest(ID);
  for (size_t i = 0; i < c.size(); i++) {
    r.push_back(request_distance(ID, c[i]));
    b.push_back(request_bearing(ID, c[i]));
  }
}
