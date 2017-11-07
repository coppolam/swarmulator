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
	vector<float> weights;
	vector<int> waiting;

	// float get_preferred_bearing(const vector<float> &bdes, const float v_b);
	// float get_attraction_velocity(float u);
	// float get_attraction_velocity_bearingbased(float u, float b);
	// void get_velocity_command_radial(const int &ID, const vector<float> &q, float &vx, float &vy);
	virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y)=0;

	int get_waitingtime(int ID);
	float saturate(float f);
	
	void assess_situation(int ID, vector<float> &q_old);
	void fill_template(vector<float> &q, const float b_i, const float u, float dmax);

	void set_saturation(const float &saturation_limits);
	inline void remove_saturation() {saturation = false;};
};


#endif /*CONTROLLER_H*/
