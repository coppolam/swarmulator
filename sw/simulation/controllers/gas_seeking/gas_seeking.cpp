#include "gas_seeking.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"
#include <tuple>
#include "randomgenerator.h"

void gas_seeking::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  random_generator rg;
  /*** Put your controller here ***/
  std::vector<float> state = s.at(ID)->state;
  s.at(ID)->laser_ranges.clear();
  agent_pos.x = state[1];
  agent_pos.y = state[0];


  s.at(ID)->laser_pnts.clear();
  for (int i =0; i<4; i++)
	{
    laser_ray ray;
    ray.heading = laser_headings[i];
    // if( ray.heading < -M_PI)
    // {
    //   ray.heading+= M_PI*2;
    // }
    // else if (ray.heading>M_PI)
    // {
    //   ray.heading-= M_PI*2;
    // }

		laser_rays.push_back(ray);
	}

  for (int i = 0; i<4; i++)
  {
    get_laser_reads(laser_rays[i],ID);
  }
  float threshold = 0.3;
  std::vector<float> ranges = s.at(ID)->laser_ranges;
  if(std::min({ranges[0],ranges[1],ranges[3]})>threshold )
  { 
    v_x = 1.0;
    v_y =0;
    // v_x = 1.0;
    // v_y= 0.0;

  }
  else{
    v_x = 0.0;
    v_y = 1.0;
    // s.at(ID)->state[6] = rg.uniform_float(-M_PI,M_PI);
    // s.at(ID)->state[6] += 0.1;
    
  }
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
  float heading = s.at(ID)->get_orientation() + ray.heading;
  rotate_xy(0,environment.env_diagonal,-heading,laser_point.x,laser_point.y);
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

    // bool on_wall = false;
    // Point intersect;
    std::tuple<bool,Point> intersect_return;
    // if( doIntersect(agent_pos,laser_point,wall_start,wall_end))
    // {
    intersect_return = getIntersect(agent_pos,laser_point,wall_start,wall_end);
    bool on_wall = std::get<0>(intersect_return);
    Point intersect = std::get<1>(intersect_return);

    if( on_wall == true){
      ray.intersection_points.push_back(intersect);
      ray.intersecting_walls.push_back(environment.walls[i]);
      s.at(ID)->intersect_walls.push_back(environment.walls[i]);
    }
  }

  if ( ray.intersection_points.size() > 0 )
  {
    for (uint i=0; i<ray.intersection_points.size();i++)
    {
      ray.distances.push_back(getDistance(agent_pos,ray.intersection_points[i]));
    }

    int idx = std::distance(ray.distances.begin(),std::min_element(ray.distances.begin(),ray.distances.end()));
    ray.range = ray.distances[idx];
    s.at(ID)->laser_ranges.push_back(ray.range);
    std::vector<float> v = {ray.intersection_points[idx].x,ray.intersection_points[idx].y};
    s.at(ID)->laser_pnts.push_back(v);
    
  }

  // d.segment(agent_position.x,agent_position.y,laser_point.x,laser_point.y);
}
