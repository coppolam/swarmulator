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
    ray.heading = state[6] + laser_headings[i];
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
  Point laser_point,wall_start, wall_end, agent_pos;
  std::vector<float> state = s.at(ID)->state;
  rotate_xy(0,environment.env_diagonal,ray.heading,laser_point.x,laser_point.y);
  laser_point.x += state[1];
  laser_point.y += state[0];

  agent_pos.x = state[1];
  agent_pos.y = state[0];



  for(uint i = 0; i<environment.walls.size();i++)
  {
    wall_start.x = environment.walls[i][0];
    wall_start.y = environment.walls[i][1];
    wall_end.x = environment.walls[i][2];
    wall_end.y = environment.walls[i][3];

    // Point intersect = getIntersect(agent_pos,laser_point,wall_start,wall_end);

  // terminalinfo::debug_msg("Showing all walls:");
  // for (uint i =0; i<environment.walls.size();i++)
  // {
  //   terminalinfo::debug_msg("Wall: "+std::to_string(environment.walls[i][0])+" " +std::to_string(environment.walls[i][1])+" " +std::to_string(environment.walls[i][2])+" " +std::to_string(environment.walls[i][3]));
  // }

    if( doIntersect(agent_pos,laser_point,wall_start,wall_end))
    // if (on_wall == true)
    {
      terminalinfo::debug_msg("Wall: "+std::to_string(wall_start.x)+" "+std::to_string(wall_start.y)+" "+std::to_string(wall_end.x)+" "+std::to_string(wall_end.y));
      ray.intersection_points.push_back(getIntersect(agent_pos,laser_point,wall_start,wall_end));
    }
  }

  for (uint i=0; i<ray.intersection_points.size();i++)
  {
    ray.distances.push_back(getDistance(agent_pos,ray.intersection_points[i]));
  }

  int idx = std::distance(ray.distances.begin(),std::min_element(ray.distances.begin(),ray.distances.end()));
  ray.range = ray.distances[idx];
  
  std::vector<float> v = {ray.intersection_points[idx].x,ray.intersection_points[idx].y};
  s.at(ID)->laser_pnts.push_back(v);

  // d.segment(agent_position.x,agent_position.y,laser_point.x,laser_point.y);
}