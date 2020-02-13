#ifndef BTFILE_H
#define BTFILE_H

#include "btCommon.h"

namespace BT
{

/**
 * @brief Load a behavior tree file
 *
 * @param file The name of the file
 * @return composite* The composite tree structure
 */
BT::composite *loadFile(const char *file);

/**
 * @brief
 *
 * @param file
 * @param task
 * @return true Success
 * @return false Failure
 */
bool saveFile(const char *file, composite *task);

/**
 * @brief
 *
 * @param myfile
 * @param parent
 * @param tabs
 */
void writeComposite(std::ofstream &myfile, composite *parent, size_t tabs);

/**
 * @brief
 *
 * @param Parent
 * @param line
 * @return node*
 */
node *newTask(node *Parent, std::string line);

/**
 * @brief
 *
 * @param line
 * @return node*
 */
node *decodeLine(std::string line);

/**
 * @brief
 *
 * @param task
 * @return std::string
 */
std::string codeLine(node *task);
}

#endif // BTFILE_H
