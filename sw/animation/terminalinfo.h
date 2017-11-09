#ifndef TERMINALINFO_H
#define TERMINALINFO_H

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <iostream>

using namespace std;

class terminalinfo
{

public:
  terminalinfo() {}; // Make two versions for random initialization
  ~terminalinfo() {};

  void debug_msg(string str);
  void info_msg(string str);
};

#endif  /* TERMINALINFO_H */