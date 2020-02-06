/*
 * Behaviour.cpp
 *
 *  Created on: 18 Feb 2015
 *      Author: Kirk Scheper
 *      Email:  kirkscheper@gmail.com
 */

#include "behaviour.h"
#include <string.h>

namespace BT
{

// ******************************************* //

// Evaluate node
BT_Status node::tick(blackboard *BLKB)
{
  BT_Status returnStatus;
  // If not initialised, initialize
  if (m_eStatus == BH_INVALID) {
    onInitialise();
  }
  
  // Run node behaviour
  m_eStatus = update(BLKB);
  returnStatus = m_eStatus;

  // If completed, run terminate code and reinitialise
  if (m_eStatus != BH_RUNNING) {
    onTerminate(m_eStatus);
    m_eStatus = BH_INVALID;
  }

  tick_counter++;

  return returnStatus;
}

}
