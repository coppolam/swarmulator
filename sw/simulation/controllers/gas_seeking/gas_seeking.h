#ifndef GAS_SEEKING_H
#define GAS_SEEKING_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "auxiliary.h"

class laser_ray
{
	public:
		int num_walls; //number of walls that the laser ray intersects with
		float heading; //heading in ENU frame
		Point start_pos; //start position of the ray
		std::vector<std::vector<float>> walls; //all walls that the laser ray intersects with
		std::vector<Point> intersection_points; //intersection points with walls
		std::vector<float> distances;//distances to walls of intersection
		float range; //final outcome, the measured range
};

class gas_seeking: public Controller
{
public:
	gas_seeking():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
	Point agent_pos, laser_point;
	void get_laser_reads(laser_ray ray, const uint16_t ID);
	float laser_headings[4] = {0,M_PI/2.,M_PI,(3./2.)*M_PI};
	std::vector<laser_ray> laser_rays;


};



#endif /*GAS_SEEKING_H*/
