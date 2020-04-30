#include "txtwrite.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include <sstream>
#include "agent.h"
#include "main.h"
#include <iomanip>      // std::setprecision

txtwrite::txtwrite() {}

void txtwrite::setfilename(const string &s)
{
  filename = s;
}

void txtwrite::txtwrite_state(ofstream &logfile)
{
  stringstream t;
  t << simtime_seconds;
  vector<Agent *> state_buff = s;

  for (uint8_t i = 0; i < s.size(); i++) {
    logfile << t.str() << " " // time
            << i + 1 << " "; // ID
    // log states
    for (uint8_t j = 0; j < 2; j++) {
      logfile << state_buff[i]->state.at(j) << " ";
    }
    logfile << endl;
  }
}

void txtwrite::txtwrite_summary(ofstream &logfile)
{
  bool killflag = false;

#if defined(REMAIN_CONNECTED) || defined(CHECK_HAPPY)
  OmniscientObserver o;
#endif

#ifdef REMAIN_CONNECTED
  if (!(o.connected_graph_range(rangesensor))) {
    logfile << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << endl;
    terminalinfo::debug_msg("Swarm broke");
    killflag = true;
  }
#endif

#ifdef CHECK_HAPPY
  if (o.check_happy()) {
    terminalinfo::debug_msg("Pattern completed");
    killflag = true;
  }
#endif

  if (killflag) {
    program_running = false;
  }
}
