#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <algorithm>

using namespace std;

class Controller {
private:
	
	float f_attraction(float u);	
	float f_attraction_bearing(float u, int b);
	float f_repulsion(float u);
	float f_extra(float u);
	bool  saturation;
	float saturation_limits;
	int   k = 0;
	bool  set = true;
    time_t timer;

public:
	vector<float> weights;
	vector<int> waiting;

	Controller();
	~Controller();

	int get_bearing_velocity(const vector<float> &bdes, const float v_b);
	float get_attraction_velocity(float u, int b);
	float get_velocity_command_radial(const int &ID, const vector<float> &q, float &vx, float &vy);
	float get_velocity_command_cartesian(int ID, int dim);

	int get_waitingtime(int ID);
	float saturate(float f);
	
	void assess_situation(int ID, vector<float> &q_old);

	void  fill_template(vector<float> &q, const float b_i, const float u, float dmax);

	void  set_weights(const vector<float> &w);
	void  set_saturation(const float &lim);
	void  remove_saturation()	{saturation = false;};
	void  set_k(int i)			{k = i;};
};


#endif /*CONTROLLER_H*/
