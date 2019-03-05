#include "txtwrite.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"

#include "agent.h"
#include "main.h"

void txtwrite::setfilename(const string &s)
{
  filename = s;
}

void txtwrite::txtwrite_state(ofstream &logfile)
{
  stringstream t;
  t << simtime_seconds;
  vector<Agent *> state_buff = s;

  for (uint8_t i = 0; i < nagents; i++) {
    logfile << t.str() << " " // time
            << i + 1 << " " // ID
            << state_buff[i]->state.at(0) << " " // p_x global
            << state_buff[i]->state.at(1) << " " // p_y global
            << state_buff[i]->state.at(2) << " " // v_x local frame
            << state_buff[i]->state.at(3) << " " // v_y local frame
            << state_buff[i]->state.at(6) << endl; // orientation global
  }
}

void txtwrite::txtwrite_summary(ofstream &logfile)
{
  bool killflag = false;

#if defined(REMAIN_CONNECTED) || defined(CHECK_HAPPY)
  OmniscientObserver o;
  terminalinfo ti;
#endif

#ifdef REMAIN_CONNECTED
  if (!(o.connected_graph_range(rangesensor))) {
    logfile << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
    ti.debug_msg("Swarm broke");
    killflag = true;

  }
#endif

#ifdef CHECK_HAPPY
  if (o.check_happy()) {
    terminalinfo ti;
    ti.debug_msg("Pattern completed");
    killflag = true;
  }
#endif

  if (killflag) {
    program_running = false;
  }
}