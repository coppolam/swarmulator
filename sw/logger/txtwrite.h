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

using namespace std;

/**
 * Class to write data to a txt file, used for logging purposes
 */
class txtwrite
{
public:
  string filename;

  txtwrite();

  /**
   * Set the name of the txt file to be used
   *
   * @param s The name of the txt file
   */
  void setfilename(const string &s);

  /**
   * Write the state of a robot to a txt file
   *
   * @param logfile The txt file ID (from ofstream)
   */
  void txtwrite_state(ofstream &logfile);

  /**
   * Write other properties. The definition of this function can be modified to different needs.
   * TODO: Generalize how this handled.
   *
   * @param logfile The txt file ID (from ofstream)
   */
  void txtwrite_summary(ofstream &logfile);
};


#endif /*WRITE_H*/