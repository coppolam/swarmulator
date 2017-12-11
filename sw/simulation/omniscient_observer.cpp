#include "omniscient_observer.h"
#include "main.h"

// struct indexed_array;
/* Structure holding values and indexes of an array, used for sorting while keeping original index*/
bool printed = false;

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
  for (int i = 0; i < nagents; i++) {
    dm[i].values = (sqrt(
                      pow(s[i].get_position(0) - s[ID].get_position(0), 2.0)
                      + pow(s[i].get_position(1) - s[ID].get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(nagents, dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (int i = 1 ; i < nagents ; i++) {
    ind.push_back(dm[i].index);
  }

  return ind;
}

vector<int> OmniscientObserver::request_closest_inrange(uint8_t ID, float range)
{
  indexed_array dm[nagents - 1];
  vector<int> ind;
  for (int i = 0; i < nagents; i++) {
    dm[i].values = (sqrt(
                        pow(s[i].get_position(0) - s[ID].get_position(0), 2.0)
                      + pow(s[i].get_position(1) - s[ID].get_position(1), 2.0)
                    ));
    dm[i].index = i;
  }

  array_sortmintomax_index(nagents, dm);

  // Start from one to eliminate youself from the list (because you are 0 distance)
  for (int i = 1 ; i < nagents ; i++) {
    if (dm[i].values < range) {
      ind.push_back(dm[i].index);
    }
  }
  return ind;
}

void mat_print(int n_row, int n_col, bool a[])
{
  int row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      if (a[ridx]) {
        cout << 1 << " ";
      } else {
        cout << 0 << " ";
      }

    }
    cout << ";" << endl;
  }
}

void OmniscientObserver::adjacency_matrix()
{
  bool mat[nagents * nagents];

  for (uint8_t i = 0; i < nagents; i++) {
    vector<int> vr = request_closest(i);
    vector<int> v(vr.begin(), vr.begin() + knearest); // cut to knearest

    for (uint8_t j = 0; j < nagents; j++) {
      if (i == j) {
        mat[i * nagents + j] = false;
      } else if (find(v.begin(), v.end(), j) != end(v)) { // TODO: Fix to only look at closest k
        mat[i * nagents + j] = true;
      } else {
        mat[i * nagents + j] = false;
      }

    }
  }

  if (!printed &&
      ((simulation_realtimefactor * simulation_time / 1000000.0) > 40.0)) {
    cout << "A = [ " ;
    mat_print(nagents, nagents, mat);
    printed = true;
    cout << " ] " << endl;
  }
}

float OmniscientObserver::get_centroid(uint8_t dim)
{
  float c = 0;
  for (uint8_t i = 0; i < nagents; i++) {
    c += s[i].get_position(dim) / (float)nagents;
  }
  return c;
}

float OmniscientObserver::request_distance_dim(uint8_t ID, uint8_t ID_tracked, uint8_t dim)
{
  return s[ID_tracked].get_position(dim) - s[ID].get_position(dim);
}

float OmniscientObserver::request_distance(uint8_t ID, uint8_t ID_tracked)
{
  float u = 0;
  for (uint8_t i = 0; i < 2; i++) {
    float dd = s[ID_tracked].get_position(i) - s[ID].get_position(i);
    u += pow(dd, 2);
  }
  return sqrt(u);
}

float OmniscientObserver::request_bearing(uint8_t ID, uint8_t ID_tracked)
{
  return atan2(request_distance_dim(ID, ID_tracked, 1), request_distance_dim(ID, ID_tracked, 0));
}