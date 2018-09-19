#include "terminalinfo.h"

void terminalinfo::debug_msg(string str)
{
#ifdef DEBUG // Defined in the makefile!
  cout << "\e[01;31m[DEBUG]: \e[0m" << str << endl;
#endif
}

void terminalinfo::info_msg(string str)
{
#ifdef INFO // Defined in the makefile!
  cout << "\e[01;34m[INFO]: \e[0m" << str << endl;
#endif
}