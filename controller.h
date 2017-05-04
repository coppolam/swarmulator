#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>

using namespace std;

class Controller {
private:
	
	float f_attraction(float u);	
	float f_attraction_bearing(float u, float b);
	float f_repulsion(float u);
	float f_extra(float u);
	bool  saturation;
	float saturation_limits;
	int   k = 0;
	bool  set = true;
public:
	vector<float> weights;
	Controller();
	~Controller();
	
	float get_individual_command(float u,float b);
	float get_velocity_command_radial(int ID, int dim);
	float get_velocity_command_cartesian(int ID, int dim);
	float saturate(float f);

	void  set_weights(const vector<float> &w);
	void  set_saturation(const float &lim);
	void  remove_saturation()	{saturation = false;};
	void  set_k(int i)			{k = i;};
};


#endif /*CONTROLLER_H*/
