#ifndef AGENT_OBSERVER_H
#define AGENT_OBSERVER_H

#include <vector>
#include <stdio.h>
#include <iostream>

class AgentObserver {
private:
	
public:
	AgentObserver(){};
	~AgentObserver(){};
	std::vector<float> update(const int &ID, const int &dim);
};


#endif /*AGENT_OBSERVER_H*/