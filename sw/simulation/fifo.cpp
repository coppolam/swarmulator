#include "fifo.h"
#include "settings.h"
#include "main.h"
#include <sstream> // std::stringstream, std::stringbuf
#include <unistd.h> // write
#include <sys/types.h> // fifo related
#include <sys/stat.h> // fifo related
#include <fcntl.h> // fifo related

fifo::fifo(std::string id)
{
  fifo::open("/tmp/swarmulator_" + id);
  terminalinfo::debug_msg("Pipe set to " + id);
};

void fifo::open(std::string file)
{
  char const *bt_fifo_write = file.c_str();
  if (access(bt_fifo_write, F_OK) == -1) {
    mkfifo(bt_fifo_write, 0666);
  }
  fifo_write_id = ::open(bt_fifo_write, O_RDWR | O_NONBLOCK);
}

bool fifo::send(float f)
{
  uint16_t size = 8;
  char msg[size];
  sprintf(msg, "%f", f);
  return write(fifo_write_id, (char *)msg, size * sizeof(char));
}
