#ifndef BEHAVIOR_TREE_H
#define BEHAVIOR_TREE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

using namespace std;

// BT library
#include "bt/btCommon.h"
#include "bt/btFile.h"
using namespace BT;

class behavior_tree: public Controller
{
// public:
// 	// Inputs
// 	float* sensor;

// public:		////////////////////
//     float wheelSpeed0;
//     float wheelSpeed1;

public:
	behavior_tree();
	composite *tree;
	blackboard BLKB;
	virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);
};

#endif /*BEHAVIOR_TREE_H*/
