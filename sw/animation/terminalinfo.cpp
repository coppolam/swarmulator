#include "terminalinfo.h"

void terminalinfo::debug_msg(string str)
{
#ifdef DEBUG // To be defined in the makefile so as to be applicable globally.
  cout << "\e[01;31m[DEBUG]: \e[0m" << str << endl;
#endif
}

void terminalinfo::info_msg(string str)
{
#ifdef INFO // To be defined in the makefile so as to be applicable globally.
  cout << "\e[01;34m[INFO]: \e[0m" << str << endl;
#endif
}