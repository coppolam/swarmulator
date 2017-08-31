#ifndef TERMINALINFO_H
#define TERMINALINFO_H

void debug_msg(string str);
void info_msg(string str);

// #define DEBUG_MSG(str) do { cout << "\e[01;31m[DEBUG]: \e[0m" << str << endl; } while (false)
// #else
// #define DEBUG_MSG(str) do { } while (false)
// #endif

// #ifdef INFO
// #define INFO_MSG(str) do { cout << "\e[01;33m[INFO]: \e[0m" << str << endl; } while (false)
// #else
// #define INFO_MSG(str) do { } while (false)
// #endif

#endif  /* TERMINALINFO_H */