#include <sstream>
#include <iomanip>      // std::setprecision

#include "txtwrite.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include "agent.h"
#include "main.h"
#include "fitness_functions.h"

using namespace std;

txtwrite::txtwrite() {}

void txtwrite::setfilename(const string &s)
{
  filename = s;
}

void txtwrite::txtwrite_state(ofstream &logfile)
{
  std::stringstream t;
  t << simtime_seconds;
  vector<Agent *> state_buff = s;
#ifdef ESTIMATOR
  float f = evaluate_fitness();
#endif
  for (uint16_t i = 0; i < s.size(); i++) {
    logfile << t.str() << " " // time
            << i + 1 << " "; // ID
    // log states
    for (uint16_t j = 0; j < 2; j++) {
      logfile << state_buff[i]->state.at(j) << " ";
    }
#ifdef ESTIMATOR
    if (pr.estimator_active) {
      logfile << pr.s_kp1[i] << " " << f;
    }
#endif
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
