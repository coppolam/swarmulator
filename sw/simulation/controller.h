#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <algorithm>
#include <stdint.h>

using namespace std;

class Controller {

public:
	Controller();
	~Controller();

	bool  saturation;
	float saturation_limits;

	void set_saturation(const float &saturation_limits);
	float saturate(float f);

	virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y) = 0;
};


#endif /*CONTROLLER_H*/
