#include "omniscient_observer.h"
#include "trigonometry.h"
#include "main.h"

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

std::vector<uint> OmniscientObserver::request_closest(const uint16_t &ID)
{
  indexed_array dm[s.size() - 1];
  std::vector<uint> ind;
  for (uint16_t i = 0; i < s.size(); i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(s.size(), dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint16_t i = 1; i < s.size(); i++) {
    ind.push_back(dm[i].index);
  }

  return ind;
}

std::vector<uint> OmniscientObserver::request_closest_inrange(const uint16_t &ID, const float &range)
{
  indexed_array dm[s.size() - 1];
  std::vector<uint> ind;
  for (uint16_t i = 0; i < s.size(); i++) {
    dm[i].values = (sqrt(
                      pow(s[i]->get_position(0) - s[ID]->get_position(0), 2.0)
                      + pow(s[i]->get_position(1) - s[ID]->get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(s.size(), dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (uint16_t i = 1; i < s.size(); i++) {
    if (dm[i].values < range) {
      ind.push_back(dm[i].index);
    }
  }
  return ind;
}

bool OmniscientObserver::check_happy()
{
  bool happy = true;
  for (uint16_t ID = 0; ID < s.size(); ID++) {
    if (!s[ID]->controller->happy) {
      return false;
    }
  }

  return happy;
}

bool OmniscientObserver::connected_graph_range(const float &range)
{
  Graph g(s.size());

  for (uint16_t i = 0; i < s.size(); i++) {
    for (uint16_t j = 0; j < s.size(); j++) {
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

float OmniscientObserver::get_centroid(const uint16_t &dim)
{
  float c = 0;
  for (uint16_t i = 0; i < s.size(); i++) {
    c += s[i]->get_position(dim) / (float)s.size();
  }
  return c;
}

float OmniscientObserver::request_distance_dim(const uint16_t &ID, const uint16_t &ID_tracked, const uint16_t &dim)
{
  return s[ID_tracked]->get_position(dim) - s[ID]->get_position(dim);
}

float OmniscientObserver::request_distance(const uint16_t &ID, const uint16_t &ID_tracked)
{
  float u = 0;
  for (uint16_t i = 0; i < 2; i++) {
    float dd = s[ID_tracked]->get_position(i) - s[ID]->get_position(i);
    u += pow(dd, 2);
  }
  float noise = rg.gaussian_float(0.0, NOISE_R);
  return sqrt(u) + noise;
}

float OmniscientObserver::own_bearing(const uint16_t &ID)
{
  return s[ID]->get_orientation();
}

float OmniscientObserver::request_bearing(const uint16_t &ID, const uint16_t &ID_tracked)
{
  float noise = rg.gaussian_float(0.0, NOISE_B);
  float b = atan2(request_distance_dim(ID, ID_tracked, 1), request_distance_dim(ID, ID_tracked, 0)) + noise;
#if COMMAND_LOCAL
  return b - own_bearing(ID);
#else
  return b;
#endif

}

bool OmniscientObserver::see_if_moving(const uint16_t &ID)
{
  return s[ID]->moving;
}

void OmniscientObserver::relative_location_inrange(const uint16_t ID, const float range, std::vector<float> &r,
    std::vector<float> &b)
{
  std::vector<uint> closest = request_closest_inrange(ID, range);
  for (size_t i = 0; i < closest.size(); i++) {
    r.push_back(request_distance(ID, closest[i]));
    b.push_back(request_bearing(ID, closest[i]));
  }
}

void OmniscientObserver::relative_location(const uint16_t ID, std::vector<float> &r, std::vector<float> &b)
{
  std::vector<uint> c = request_closest(ID);
  for (size_t i = 0; i < c.size(); i++) {
    r.push_back(request_distance(ID, c[i]));
    b.push_back(request_bearing(ID, c[i]));
  }
}

bool OmniscientObserver::sense_food(const uint16_t ID, uint16_t &food_ID, float range)
{
  mtx_env.lock_shared();
  for (uint16_t i = 0; i < environment.food.size(); i++) {
    float u = 0;
    for (size_t j = 0; j < 2; j++) {
      float dd = s[ID]->get_position(j) - environment.food[i][j];
      u += pow(dd, 2);
    }
    if (sqrt(u) < range) {
      food_ID = i;
      mtx_env.unlock_shared();
      return true;
    }
  }
  mtx_env.unlock_shared();
  return false;
}

void OmniscientObserver::beacon(const uint16_t ID, float &r, float &b)
{
  mtx_env.lock_shared();
  float x = environment.beacon[0] - s[ID]->get_position(0);
  float y = environment.beacon[1] - s[ID]->get_position(1);
  cart2polar(x, y, r, b);
#if COMMAND_LOCAL
  b -= own_bearing(ID);
#endif
  mtx_env.unlock_shared();
}
