#ifndef TXTWRITE_H
#define TXTWRITE_H
#include <mutex>

#include <fstream>
#include "particle.h"
#include "agent.h"
#include "main.h"

using namespace std;

class txtwrite {
public:
	void txtwrite_state(ofstream &logfile);
	// void write_command(int ID);
};


#endif /*WRITE_H*/