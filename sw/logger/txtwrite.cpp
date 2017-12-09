#include "txtwrite.h"

void txtwrite::txtwrite_state(ofstream &logfile)
{
  for (int i = 0; i < nagents; i++) {
    logfile << i+1 << " ";
    logfile << simulation_realtimefactor *simulation_time / 1000000.0 << " "
            << s[i].state.at(0) << " "
            << s[i].state.at(1) << endl;
  }
}