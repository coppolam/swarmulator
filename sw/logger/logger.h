#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>

#include "parameters.h"
#include "txtwrite.h"

using namespace std;

void run_logger(ofstream &logfile)
{
	static txtwrite writer;

	for (int i = 0; i < nagents; i++)
	{
		writer.txtwrite_state(logfile);
		// add other potential things you want to write
		// TODO: Make a config file for this
	}

	int t_wait = (int) 1000000.0*(1.0/(logger_updatefreq*simulation_realtimefactor));
	this_thread::sleep_for(chrono::microseconds( t_wait ));
}

void start_logger(int argc, char* argv[])
{
	string filename = "log.txt";
	ofstream logfile;
	logfile.open (filename.c_str());
	logfile.precision(3);

	while(true) 
		{ run_logger(logfile); };
}

#endif /*SIMULATION_H*/