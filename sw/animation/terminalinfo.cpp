#include "terminalinfo.h"
#include "main.h"

void terminalinfo::debug_msg(string str)
{
#ifdef VERBOSE // Defined in the makefile!
  cout << "\e[01;36m[DEBUG]: \e[0m" << str << endl;
#endif
}

void terminalinfo::debug_msg(string str, int ID)
{
#ifdef VERBOSE // Defined in the makefile!
  cout << "\e[01;36m[DEBUG]: \e[0m" << "Robot " << ID << ":\t" << str << endl;
#endif
}

void terminalinfo::info_msg(string str)
{
#ifdef VERBOSE // Defined in the makefile!
  cout << "\e[01;34m[INFO]: \e[0m" << str << endl;
#endif
}

void terminalinfo::warning_msg(string str)
{
#ifdef VERBOSE
  cout << "\e[01;33m[WARNING]: \e[0m" << str << endl;
#endif
}

void terminalinfo::error_msg(string str)
{
  cout << "\e[01;31m[ERROR]: \e[0m" << str << endl;
  program_running = false;
}