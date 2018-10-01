#ifndef TXTWRITE_H
#define TXTWRITE_H

#include <mutex>
#include <fstream>

#include "drawingparams.h"

using namespace std;

/**
 * Class to write data to a txt file, used for logging purposes
 */
class txtwrite
{
public:
  string filename;

  /**
   * Set the name of the txt file to be used
   */
  void setfilename(const string &s);

  /**
   * Write the state of a robot to a txt file
   */
  void txtwrite_state(ofstream &logfile);

  /**
   * Write other properties
   */
  void txtwrite_summary(ofstream &logfile);
};


#endif /*WRITE_H*/