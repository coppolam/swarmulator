#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <ctime>

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

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

    return buf;
}

void start_logger(int argc, char* argv[])
{
	// time_t t = time(0);   // get time now
 //    struct tm * now = localtime( & t );
	string filename = "logs/log_" + currentDateTime() + ".txt";
	// << (now->tm_year + 1900) << '-' 
 //         << (now->tm_mon + 1) << '-'
 //         <<  now->tm_mday
 //         << endl

	ofstream logfile;
	logfile.open (filename.c_str());
	logfile.precision(3);

	while(true) 
		{ run_logger(logfile); };
}

#endif /*SIMULATION_H*/