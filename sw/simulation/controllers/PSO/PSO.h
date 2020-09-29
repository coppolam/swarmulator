#ifndef PSO_H
#define PSO_H

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
		std::vector<std::vector<float>> intersecting_walls; //the walls it's intersecting with
		std::vector<float> intersect_wall; //the wall that it's intersecting with
};

class PSO: public Controller
{
public:
	PSO():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &psi, float &v_x);
	virtual void animation(const uint16_t ID);
	void get_laser_reads(laser_ray ray, const uint16_t ID);
	float get_heading_to_point(Point agent, Point goal);
	float laser_headings[4] = {M_PI_2,0,3*M_PI_2,M_PI};
	
	float iteration_start_time = 0.0;
	float local_psi, local_vx;

	// configuration parameters for PSO
	float rand_p = 0.4;
	float omega = 0.2;
	float phi_p = 0.5;
	float phi_g = 1.5;
	float yaw_incr = 0.1;
	float update_time = 5.0;
	float dist_reached_goal = 0.5;

	bool wall_following = false;
	float ranger_threshold = 1.0; // if one of the rangers is below this threshold, we start wall-following
	Point agent_pos, goal, random_point;
	std::vector<laser_ray> laser_rays;
	bool rotating_wall_follow = true;
	bool moving_wall_follow = false;
	bool rotation_dir_decided = false;
	bool rotate_right = true;
	
};

#endif /*PSO_H*/
