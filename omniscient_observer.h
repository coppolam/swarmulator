#ifndef OMNISCIENT_OBSERVER_H
#define OMNISCIENT_OBSERVER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::sort
#include <mutex>

using namespace std;

struct indexed_array {
    float values;
    int index;
};

/* Collects data from all the agents and spits out the k-nearest for each one */
class OmniscientObserver {
	// void update(const int &ID, indexed_array &dm);
public:
	OmniscientObserver(){};
	~OmniscientObserver(){};
	vector<int> request_closest(int ID); // request IDs of closest k neighbours and for your ID
	float request_distance_dim(int ID, int ID_tracked, int dim);
	float request_distance(int ID, int ID_tracked);
	float request_bearing(int ID, int ID_tracked);
	void adjacency_matrix();
	float get_centroid(int dim);
};




#endif /*OMNISCIENT_OBSERVER_H*/