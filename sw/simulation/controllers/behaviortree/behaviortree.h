#ifndef BEHAVIORTREE_H
#define BEHAVIORTREE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

// BT library
#include "bt/btCommon.h"
#include "bt/btFile.h"
using namespace BT;

class behaviortree: public Controller
{
public:
	behaviortree();
	composite *tree;
	virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*BEHAVIORTREE_H*/
