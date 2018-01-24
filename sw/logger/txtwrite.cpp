#include "txtwrite.h"
#include "omniscient_observer.h"
#include "kill_functions.h"
#include "terminalinfo.h"

void txtwrite::setfilename(string s) {
  filename = s;
}

void txtwrite::txtwrite_state(ofstream &logfile)
{
  for (int i = 0; i < nagents; i++) {
    logfile << i+1 << " ";
    logfile << simulation_realtimefactor * simulation_time / 1000000.0 << " "
            << s[i].state.at(0) << " "
            << s[i].state.at(1) << " "
            << s[i].state.at(2) << " "
            << s[i].state.at(3) << " " << endl;
  }
}

void txtwrite::txtwrite_summary(ofstream &logfile)
{
  OmniscientObserver o;

  if (!o.connected_graph_range(1.8))
  {
    logfile << o.connected_graph_range(1.8)
            << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
    killer k;
    terminalinfo ti;
    ti.debug_msg("broke");
    k.kill_switch();
  }
}