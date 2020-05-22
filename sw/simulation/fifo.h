#ifndef FIFO_H
#define FIFO_H
#include <iostream>
#include <string>

/**
 * Handles FIFO messaging for external interfacing.
 *
 */
class fifo
{
  int fifo_write_id;

public:
  /**
   * @brief Construct a new fifo object
   *
   */
  fifo(std::string a);

  /**
   * @brief Destroy the fifo object
   *
   */
  ~fifo() {};

  /**
   * Create and open the FIFO pipe to communicate/interface with external programs, if needed.
   * By default, it is used to send the fitness.
   */
  void open(std::string);

  /**
   * @brief Send out a FIFO message to interfact with external programs.
   *
   * @param fd FIFO file ID, initiated in the beginning of the thread
   * @return int
   */
  bool send(float f);
};

#endif /*FIFO_H*/
