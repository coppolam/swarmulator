#ifndef TXTWRITE_H
#define TXTWRITE_H

#include <cstdlib> // system, NULL, EXIT_FAILURE
#include <iostream>
#include <sstream> // std::stringstream, std::stringbuf
#include <thread>
#include <future>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <mutex>
#include <fstream>
#include <sstream> // std::stringstream, std::stringbuf
#include "drawingparams.h"

/**
 * Class to write data to a txt file, used for logging purposes
 */
class txtwrite
{
public:
  std::string filename; // Name of the log file

  /**
   * @brief Construct a new txtwrite object
   *
   */
  txtwrite();

  /**
   * Set the name of the txt file to be used
   *
   * @param s The name of the txt file
   */
  void setfilename(const std::string &s);

  /**
   * Write the state of a robot to a txt file
   *
   * @param logfile The txt file ID (from ofstream)
   */
  void txtwrite_state(std::ofstream &logfile);
};


#endif /*WRITE_H*/
