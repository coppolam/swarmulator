#ifndef TERMINALINFO_H
#define TERMINALINFO_H

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <iostream>

using namespace std;

/**
 * terminalinfo is used to print info commands to the terminal.
 * These can come in the form of info messages or debug messages
 * The debug and info flags are activated (or deactivated) in the makefile
 */
class terminalinfo
{

public:
  /**
   * Constructor
   */
  terminalinfo() {};

  /**
   * Destructor
   */
  ~terminalinfo() {};

  /**
   * Print a debug message
   */
  void debug_msg(string str);

  /**
   * Print an info message
   */
  void info_msg(string str);
};

#endif  /* TERMINALINFO_H */