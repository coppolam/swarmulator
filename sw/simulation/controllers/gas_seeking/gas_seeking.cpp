#include "gas_seeking.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"

void gas_seeking::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  /*** Put your controller here ***/
  std::vector<float> state = s.at(ID)->state;
  agent_pos.x = state[1];
  agent_pos.y = state[0];
  laser_point.x = 0;
  laser_point.y = 0; 

  for (int i =0; i<4; i++)
	{
    laser_ray ray;
    ray.heading = state[6] + laser_headings[6];
		laser_rays.push_back(ray);
	}

  for (int i = 0; i<4; i++)
  {
    get_laser_reads(laser_rays[i],ID);
  }
  // get_laser_reads(agent_pos,state[6],laser_point,laser_point, ID);
  // v_x = 0.0;
  // v_y = 0;
}
void gas_seeking::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
  /*** Put the animation of the controller/sensors here ***/
}

/**
 * Helper function to model laser ranger in a specified direction
 * @param (Point) agent_position: current position of agent to consider
 * @param (float) heading: orientation of the laser ranger for the agent
 * @param (Point) wall_1: first point on the wall
 * @param (Point) wall_2: second point on the wall
*/
void gas_seeking::get_laser_reads(laser_ray ray, const uint16_t ID)
{
  //construct a point in the right direction that is outside of the environment
  Point laser_point;
  std::vector<float> state = s.at(ID)->state;
  rotate_xy(0,environment.env_diagonal,ray.heading,laser_point.x,laser_point.y);
  laser_point.x += state[1];
  laser_point.y += state[0];
  // std::vector<float> v = {laser_point.x,laser_point.y};
  // s.at(ID)->laser_pnts.push_back(v);

  // d.segment(agent_position.x,agent_position.y,laser_point.x,laser_point.y);
}