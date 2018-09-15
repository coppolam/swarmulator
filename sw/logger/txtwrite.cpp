#include "txtwrite.h"
#include "omniscient_observer.h"
#include "kill_functions.h"
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
    logfile << i + 1 << " ";
    logfile << t.str() << " "; // divide by 1000000.0
    logfile << state_buff[i]->state.at(0) << " "
            << state_buff[i]->state.at(1) << " "
            << state_buff[i]->state.at(2) << " "
            << state_buff[i]->state.at(3) << endl;
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

  if (killflag){
    killer k;
    k.kill_switch();
  }
}