#ifndef TXTWRITE_H
#define TXTWRITE_H

#include <mutex>
#include <fstream>

#include "drawingparams.h"

using namespace std;

class txtwrite
{
public:
  string filename;
  void setfilename(const string &s);
  void txtwrite_state(ofstream &logfile);
  void txtwrite_summary(ofstream &logfile);
};


#endif /*WRITE_H*/