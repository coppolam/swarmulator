#include "terminalinfo.h"
#include "main.h"

void terminalinfo::debug_msg(std::string str)
{
#ifdef VERBOSE // Defined in the makefile!
  std::cout << "\e[01;36m[DEBUG]: \e[0m" << str << std::endl;
#endif
}

void terminalinfo::debug_msg(std::string str, int ID)
{
#ifdef VERBOSE // Defined in the makefile!
  std::cout << "\e[01;36m[DEBUG]: \e[0m" << "Robot " << ID << ":\t" << str << std::endl;
#endif
}

void terminalinfo::info_msg(std::string str)
{
#ifdef VERBOSE // Defined in the makefile!
  std::cout << "\e[01;34m[INFO]: \e[0m" << str << std::endl;
#endif
}

void terminalinfo::info_msg(std::string str, int ID)
{
#ifdef VERBOSE // Defined in the makefile!
  std::cout << "\e[01;34m[INFO]: \e[0m" << "Robot " << ID << ":\t" << str << std::endl;
#endif
}

void terminalinfo::warning_msg(std::string str)
{
#ifdef VERBOSE // Defined in the makefile!
  std::cout << "\e[01;33m[WARNING]: \e[0m" << str << std::endl;
#endif
}

void terminalinfo::error_msg(std::string str)
{
  std::cout << "\e[01;31m[ERROR]: \e[0m" << str << std::endl;
  program_running = false;
}