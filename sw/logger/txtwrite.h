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

#define BTEVO_FIFO_READ "/tmp/btevo_fifo"
#define BTEVO_FIFO_WRITE "/tmp/btevo_fifo"

using namespace std;

/**
 * Class to write data to a txt file, used for logging purposes
 */
class txtwrite
{
public:
  string filename;
  // create and open the FIFO (named pipe)
  char const *bt_fifo_read  = BTEVO_FIFO_READ;
  char const *bt_fifo_write = BTEVO_FIFO_WRITE;
  int fd_read;
  int fd_write;

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