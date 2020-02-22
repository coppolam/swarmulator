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
   *
   * @param str Debug string to print
   */
  static void debug_msg(string str);

  /**
   * Print a debug message
   *
   * @param str Debug string to print
   * @param ID Robot ID
   */
  static void debug_msg(string str, int ID);

  /**
   * Print an info message
   *
   * @param str Info string to print
   */
  static void info_msg(string str);
};

#endif  /* TERMINALINFO_H */