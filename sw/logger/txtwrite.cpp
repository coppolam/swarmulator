#include "txtwrite.h"
#include "omniscient_observer.h"
#include "terminalinfo.h"
#include <sstream>
#include "agent.h"
#include "main.h"

int send_fifo(int fd) {
  float fitness = nagents;
  cout << "running with " << nagents << "agents" << endl;
  uint8_t size = 8;
  char msg[size];
  sprintf(msg, "%f", fitness);
  return write(fd, (char *)msg, size * sizeof(char));
}

txtwrite::txtwrite()
{
  if (access(bt_fifo_read, F_OK) == -1) {
    mkfifo(bt_fifo_read, 0666);
  }

  if (access(bt_fifo_write, F_OK) == -1) {
    mkfifo(bt_fifo_write, 0666);
  }

  fd_write = open(bt_fifo_write, O_RDWR | O_NONBLOCK);
  fd_read  = open(bt_fifo_read, O_RDWR | O_NONBLOCK);
}

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
            << i + 1 << " "; // ID
    // log states
    for (uint8_t j = 0; j < 4; j++) {
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

#ifdef FITNESS
  if (o.check_happy()) {
    terminalinfo ti;
    ti.debug_msg("Pattern completed");
    killflag = true;
  }
#endif
  send_fifo(fd_write);

  if (killflag) {
    program_running = false;
  }
}
