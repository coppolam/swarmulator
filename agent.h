#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

class Agent {
	vector<float> inputs;
	vector<float> actions;
public:
	int ID;
	Controller controller;
	vector<float> outputs;
	vector<float> state;
	Agent(int i, const vector<float> &s); // Make two versions for random initialization
	~Agent();
	vector<float> get_states();
	void select_action();
	virtual void update_position()=0;
	float get_position(int dim);
	int get_ID();
};

#endif /*AGENT_H*/
