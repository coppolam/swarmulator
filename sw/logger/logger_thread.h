#ifndef LOGGER_THREAD_H
#define LOGGER_THREAD_H

#include <fstream>
#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <ctime>

#include "main.h"
#include "txtwrite.h"

bool logger_running = false;
float simtime_seconds_old;
/**
 * Run the logger
 *
 * @param logfile The file ID (ofstream)
 * @param filename The name of the logfile
 */
void run_logger(std::ofstream &logfile, std::string filename)
{
  static txtwrite writer;
  if (!logger_running) {
    terminalinfo::info_msg("Logger started.");
    writer.setfilename(filename); // Set the filename
    logger_running = true; // Set logger running to true
  }

  // Write the logfile
  if (!paused && simtime_seconds > simtime_seconds_old + 1. / param->logger_updatefreq()) {
    mtx.lock();
    writer.txtwrite_state(logfile);
    writer.txtwrite_summary(logfile);
    simtime_seconds_old = simtime_seconds;
    mtx.unlock();
  }
  if (!program_running) {std::terminate();}
}

/**
 * Logger thread that logs the simulation to a txt file
 */
void main_logger_thread()
{
  // Log filename
  std::string filename;
  filename = "logs/log_" + identifier + ".txt";

  // Open the logfile for writing
  std::ofstream logfile;
  logfile.open(filename.c_str());

  simtime_seconds_old = 0;
  // Initiate the logger
  while (program_running) {
    run_logger(logfile, filename);
  };
}

#endif /*LOGGER_THREAD_H*/
