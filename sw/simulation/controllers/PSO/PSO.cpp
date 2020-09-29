#include "PSO.h"
#include "draw.h"
#include "randomgenerator.h"
#include <tuple>
#include "main.h"
#include <math.h>
#include "auxiliary.h"

void PSO::get_velocity_command(const uint16_t ID, float &psi, float &v_x)
{
  /*** Put your controller here ***/
  random_generator rg;
  // load agent state
  std::vector<float> state = s.at(ID)->state;
  s.at(ID)->laser_ranges.clear();
  agent_pos.x = state[1];
  agent_pos.y = state[0];
  s.at(ID)->laser_pnts.clear();

  for (int i = 0; i<4; i++)
	{
    laser_ray ray;
    ray.heading = laser_headings[i];
		laser_rays.push_back(ray);
	}

  for (int i = 0; i<4; i++)
  {
    get_laser_reads(laser_rays[i],ID);
    
    if(s.at(ID)->laser_ranges[i] < ranger_threshold)
    { 
      wall_following = true;
    }
  }

  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);

  // update best found positions
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);
  if( gas_conc>s.at(ID)->best_agent_gas)
  {
    s.at(ID)->best_agent_gas = gas_conc;
    s.at(ID)->best_agent_pos = agent_pos;
    if (gas_conc > environment.best_gas)
    {
      environment.best_gas = gas_conc;
      environment.best_gas_pos_x = agent_pos.x;
      environment.best_gas_pos_y = agent_pos.y;
    }
  }

  if ( simtime_seconds-iteration_start_time >= update_time)
  {
    iteration_start_time = simtime_seconds;
    float r_p = rg.uniform_float(0,1);
    float r_g = rg.uniform_float(0,1);
    
    random_point = {.x = rg.uniform_float(environment.x_min,environment.x_max),.y = rg.uniform_float(environment.y_min,environment.y_max)};
    float v_x = rand_p*(random_point.x-agent_pos.x)+omega*(goal.x-agent_pos.x)+phi_p*r_p*(s.at(ID)->best_agent_pos.x-agent_pos.x)+phi_g*r_g*(environment.best_gas_pos_x-agent_pos.x);
    float v_y = rand_p*(random_point.y-agent_pos.y)+omega*(goal.y-agent_pos.y)+phi_p*r_p*(s.at(ID)->best_agent_pos.y-agent_pos.y)+phi_g*r_g*(environment.best_gas_pos_y-agent_pos.y);
    goal = {.x = agent_pos.x + v_x,.y = agent_pos.y+v_y}; 

    s.at(ID)->goal = goal;
    local_psi = get_heading_to_point(agent_pos,goal);

    local_vx = 1.0;
    

    moving_wall_follow = false;
    rotating_wall_follow = true;
    rotation_dir_decided = false;
    rotate_right = true;
    wall_following = false;
  }
  else if (simtime_seconds == 0.0)
  {
    // initial velocity for everyone
    goal = {.x = rg.uniform_float(environment.x_min,environment.x_max), .y=rg.uniform_float(environment.y_min,environment.y_max)};
    local_psi = get_heading_to_point(agent_pos,goal);
    local_vx = 1.0;
  }
  
  if (wall_following == true)
  {
    if (rotating_wall_follow)
    {
      if (rotation_dir_decided == false)
      {
        if (s.at(ID)->laser_ranges[0] <= s.at(ID)->laser_ranges[2])
        {
          rotate_right = false;
          rotation_dir_decided = true;
        }
        else 
        {
          rotate_right = true;
        }
      }

      if (s.at(ID)->laser_ranges[1] > ranger_threshold)
      {
        rotating_wall_follow = false;
        moving_wall_follow = true;
      }
      else
      {
        local_vx = 0.0;
        if(rotate_right)
        {
          local_psi += yaw_incr;
        }
        else
        {
          local_psi -= yaw_incr;
        }
      }
        
      
    }
    if (moving_wall_follow)
    {
      local_vx = 1.0;
      wall_following = false;
      if (s.at(ID)->laser_ranges[1]<ranger_threshold)
      {
        moving_wall_follow = false;
        rotating_wall_follow = true;
        rotation_dir_decided = false;
	      rotate_right = true;
      }
      else if (s.at(ID)->laser_ranges[0]>0.5*ranger_threshold && s.at(ID)->laser_ranges[1]>0.5*ranger_threshold && s.at(ID)->laser_ranges[2]>0.5*ranger_threshold &&s.at(ID)->laser_ranges[3]>0.5*ranger_threshold )
      {
        moving_wall_follow = false;
        rotating_wall_follow = true;
        rotation_dir_decided = false;
	      rotate_right = true;
        wall_following = false;
        local_psi = get_heading_to_point(agent_pos,goal);
      }
      else 
      {
        if (s.at(ID)->laser_ranges[0] < ranger_threshold)
        {
          local_psi -= yaw_incr;
        }
        else if (s.at(ID)->laser_ranges[2] < ranger_threshold)
        {
          local_psi += yaw_incr;
        }
      }
    }
  }
  
  float dist_to_goal = getDistance(goal,agent_pos);   
  if (dist_to_goal<dist_reached_goal)
  {
    local_vx = 0.0;
  }

  psi = local_psi;
  v_x = local_vx;
  }
void PSO::animation(const uint16_t ID)
{
  /*** Put the animation of the controller/sensors here ***/
  draw d;
  d.circle_loop(rangesensor);
}

/**
 * Returns the heading in radians between two points
 * @param x_agent: x_coord of agent
 * @param y_agent: y_coord of agent
 * @param x_point: x_coord of agent
 * @param y_point: y_coord of agent
*/
float PSO::get_heading_to_point(Point agent, Point goal)
{
  float delta_x = goal.x - agent.x;
  float delta_y = goal.y - agent.y;
  float psi = atan2(delta_x,delta_y);

  // place psi in [-pi,pi]
  if ( psi< M_PI)
  {
    psi = psi + 2*M_PI;
  }
  else if (psi> M_PI)
  {
    psi = psi - 2*M_PI;
  }

  return psi;
}

/**
 * Helper function to model laser ranger in a specified direction
 * @param (Point) agent_position: current position of agent to consider
 * @param (float) heading: orientation of the laser ranger for the agent
 * @param (Point) wall_1: first point on the wall
 * @param (Point) wall_2: second point on the wall
*/
void PSO::get_laser_reads(laser_ray ray, const uint16_t ID)
{
  //init
  Point laser_point,wall_start, wall_end, agent_pos;
  
  std::vector<float> state = s.at(ID)->state;
  float heading = s.at(ID)->get_orientation() + ray.heading; //global laser ray heading
  
  //construct a point in the right direction that is outside of the environment: laser_point
  rotate_xy(0,environment.env_diagonal,-heading,laser_point.x,laser_point.y);
  laser_point.x += state[1];
  laser_point.y += state[0];

  agent_pos.x = state[1];
  agent_pos.y = state[0];

  // looping through all walls to check if laser ray intersects with it and find the closest wall
  // we check if two lines intersect: (wall_start-wall_end) and (agent_pos-laser_point)
  for(uint i = 0; i<environment.walls.size();i++)
  {
    // init points to be used later
    wall_start.x = environment.walls[i][0];
    wall_start.y = environment.walls[i][1];
    wall_end.x = environment.walls[i][2];
    wall_end.y = environment.walls[i][3];


    std::tuple<bool,Point> intersect_return;
    //get intersection point and a bool if it's on the wall or not
    intersect_return = getIntersect(agent_pos,laser_point,wall_start,wall_end);

    bool on_wall = std::get<0>(intersect_return);
    Point intersect = std::get<1>(intersect_return);

    //storing intersecting walls and its intersection with the laser
    if( on_wall == true){
      ray.intersection_points.push_back(intersect);
      ray.intersecting_walls.push_back(environment.walls[i]);
      s.at(ID)->intersect_walls.push_back(environment.walls[i]);
    }
  }

  // if we found some intersection points
  if ( ray.intersection_points.size() > 0 )
  {
    // get a list of all distances to all intersection points
    for (uint i=0; i<ray.intersection_points.size();i++)
    {
      ray.distances.push_back(getDistance(agent_pos,ray.intersection_points[i]));
    }

    //arg max
    int idx = std::distance(ray.distances.begin(),std::min_element(ray.distances.begin(),ray.distances.end()));
    ray.range = ray.distances[idx];
    s.at(ID)->laser_ranges.push_back(ray.range);
    std::vector<float> v = {ray.intersection_points[idx].x,ray.intersection_points[idx].y};
    s.at(ID)->laser_pnts.push_back(v);
    
  }
  // we didn't find anything (this should be rare if not impossible), we we return the end of the projected laser beams and their size
  else
  {
    s.at(ID)->laser_ranges.push_back(environment.env_diagonal);
    std::vector<float> v = {laser_point.x,laser_point.y};
    s.at(ID)->laser_pnts.push_back(v);
  }
}