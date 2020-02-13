#ifndef ROBOTACTIONS_H
#define ROBOTACTIONS_H

#include <stdlib.h>
#include "node.h"
#include <cmath>

namespace BT
{
/**
 * @brief Get the action node
 *
 * @param action
 * @param inputs
 * @return node* of the action
 */
node *getAction(std::string action, std::vector<double> inputs);

/**
 * @brief Get the Action object
 *
 * @param func
 * @return node*
 */
node *getAction(size_t func = (size_t) - 1);

}
#endif // ROBOTACTIONS_H
