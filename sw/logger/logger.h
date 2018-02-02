#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <ctime>

#include "main.h"
#include "txtwrite.h"

using namespace std;

bool logger_running = false;

void run_logger(ofstream &logfile, string filename)
{
  static txtwrite writer;

  if (!logger_running) {
    terminalinfo ti;
    writer.setfilename(filename);
    ti.info_msg("Logger started.");
    logger_running = true;
  }

  if (!paused && simtime_seconds > 1.0) {
    mtx.try_lock();
    writer.txtwrite_state(logfile);
    writer.txtwrite_summary(logfile);
    mtx.unlock();
  }

  int t_wait = (int)1000000.0 / (logger_updatefreq * simulation_realtimefactor);
  this_thread::sleep_for(chrono::microseconds(t_wait));
}

/* Get current date/time, format is YYYY-MM-DD-hh:mm:ss */
const std::string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);

  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d-%X", &tstruct);

  return buf;
}
void start_logger(int argc, char *argv[])
{
  string filename = "logs/log_" + currentDateTime() + ".txt";

  ofstream logfile;
  logfile.open(filename.c_str());

  while (program_running) {
    run_logger(logfile, filename);
  };
}

#endif /*SIMULATION_H*/