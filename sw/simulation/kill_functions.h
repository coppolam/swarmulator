#ifndef KILL_FUNCTIONS_H
#define KILL_FUNCTIONS_H

#include <iostream>
#include <sstream> // std::stringstream

using namespace std;

class killer
{
public:
  killer() {}; // Make two versions for random initialization
  ~killer() {};

  void restart_switch(int nagents, int knearest);
  void kill_switch(void);
};
#endif /* KILL_FUNCTIONS_H */