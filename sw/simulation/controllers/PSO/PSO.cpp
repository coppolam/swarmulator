#include "PSO.h"
#include "draw.h"
#include "randomgenerator.h"
#include <tuple>
#include "main.h"
#include <math.h>
#include "auxiliary.h"
#include "omniscient_observer.h"

// main PSO logic
// PSO with constant heading, changing the particle's heading by modifying vx and vy
// this in order to maintain constant yaw for better UWB estimates
void PSO::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  /*** Put your controller here ***/
  float dt = s.at(ID)->dt;
  random_generator rg;
  std::vector<float> state = s.at(ID)->state; // load agent state
  s.at(ID)->laser_ranges.clear(); // laser ranges are emptied as they need to be reloaded
  s.at(ID)->laser_pnts.clear(); // laser points (where the lasers intersect with the environment)
  agent_pos.x = state[1]; // loading agent pos struct Point
  agent_pos.y = state[0];
  local_psi = get_heading_to_point(agent_pos,goal);
  laser_rays.clear();

  local_psi = get_heading_to_point(agent_pos,goal);
  local_vx = cosf(local_psi)*desired_velocity;
  local_vy = sinf(local_psi)*desired_velocity;
  // create ray objects
  for (int i = 0; i<4; i++)
	{
    laser_ray ray;
    ray.heading = laser_headings[i];
		laser_rays.push_back(ray);
	}

  heading_accumulator = 0.0;
  // load laser range values 1 by one and check if they are below threshold (i.e., we need to avoid something)
  bool reset_wall_following = true;
  bool determine_direction = false;
  for (int i = 0; i<4; i++)
  {
    laser_rays[i] = get_laser_reads(laser_rays[i],ID);
    
    if ( s.at(ID)->laser_ranges[i] < laser_rays[i].engage_laser_distance)
    {
      reset_wall_following = false;
      heading_accumulator += get_ray_control(laser_rays[i],dt);
      if(s.at(ID)->laser_ranges[i] < laser_rays[i].desired_laser_distance)
      {
        if (!wall_following)
        {
          determine_direction = true;
          wall_following = true;
        }
      }
    }
  }

// terminalinfo::debug_msg(std::to_string(get_safe_direction(s.at(ID)->laser_ranges,local_psi,laser_rays[0].desired_laser_distance,s.at(ID)->get_orientation())));
if (get_safe_direction(s.at(ID)->laser_ranges,local_psi,laser_rays[0].desired_laser_distance,s.at(ID)->get_orientation()))
{
  reset_wall_following = true;
}

if (reset_wall_following)
{
  wall_following = false;

  determine_direction = false;
}

if (determine_direction)
{
  if(get_follow_direction(s.at(ID)->laser_ranges,local_psi,s.at(ID)->get_orientation()))
  {
    follow_left = true;
  }
  else
  {
    follow_left = false;
  }
  determine_direction = false;
}

  // load gas concentration at current position
  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);

  // update best found agent position and best found swarm position if required
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

  // new goal is computed every 'update_time' [sec]
  if ( simtime_seconds-iteration_start_time >= update_time || getDistance(goal,agent_pos) < dist_reached_goal )
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
    local_vx = cosf(local_psi)*desired_velocity;
    local_vy = sinf(local_psi)*desired_velocity;
    wall_following = false;
  }
  else if (simtime_seconds == 0.0)
  {
    // initial velocity for everyone
    goal = {.x = rg.uniform_float(environment.x_min,environment.x_max), .y=rg.uniform_float(environment.y_min,environment.y_max)};
    local_psi = get_heading_to_point(agent_pos,goal);
    local_vx = cosf(local_psi)*desired_velocity;
    local_vy = sinf(local_psi)*desired_velocity;
  }
  // float dist_to_goal = getDistance(goal,agent_pos);   
  // if (dist_to_goal<dist_reached_goal)
  // {
  //   local_vx = 0.0;
  //   local_vy = 0.0;

  // }
    
  else if (wall_following == true)
  {
    if (follow_left)
    {
      heading_accumulator = -heading_accumulator;
    }
    local_psi = get_heading_to_point(agent_pos,goal) + heading_accumulator;
    local_vx = cosf(local_psi)*desired_velocity;
    local_vy = sinf(local_psi)*desired_velocity;
  }
  
  std::vector<uint> closest_ids = o.request_closest(ID);
  if ( closest_ids.size() > 0 )
  {
    if (get_agent_dist(ID,closest_ids[0]) < swarm_avoidance_thres)
    {
      other_agent_pos.x = s.at(closest_ids[0])->state[1];
      other_agent_pos.y = s.at(closest_ids[0])->state[0];
      float heading_to_other_agent = get_heading_to_point(agent_pos,other_agent_pos);
      float heading_away_from_agent = heading_to_other_agent - M_PI;
      if (get_safe_direction(s.at(ID)->laser_ranges,heading_away_from_agent,laser_rays[0].desired_laser_distance,s.at(ID)->get_orientation()))
      {
        local_vx += cosf(heading_away_from_agent)*0.5;
        local_vy += sinf(heading_away_from_agent)*0.5;
        float vector_size = sqrtf(powf(local_vx,2)+powf(local_vy,2));
        local_vx = local_vx/vector_size;
        local_vy = local_vy/vector_size;
      }
    }
  }

  v_x = local_vx;
  v_y = local_vy;
  }
void PSO::animation(const uint16_t ID)
{
  /*** Put the animation of the controller/sensors here ***/
  draw d;
  d.circle_loop(rangesensor);
}

float PSO::get_agent_dist(const uint16_t ID1, const uint16_t ID2)
{
  return(sqrtf(pow(s.at(ID1)->state[0]-s.at(ID2)->state[0],2)+pow(s.at(ID1)->state[1]-s.at(ID2)->state[1],2)));
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

float PSO::get_ray_control(laser_ray ray, float dt)
{
  ray.heading_error = ray.desired_laser_distance - ray.range;
  ray.heading_error_d = (ray.heading_error-ray.old_heading_error)/dt;
  ray.heading_error_i = 0.5*(ray.heading_error+ray.old_heading_error)*dt;

  ray.old_heading_error = ray.heading_error;
  float final_control = ray.heading_kp*ray.heading_error + ray.heading_kd*ray.heading_error_d + ray.heading_ki*ray.heading_error_i;
  return(final_control);
}

bool PSO::get_follow_direction(std::vector<float> ranges, float desired_heading, float agent_heading)
{
  if( desired_heading < 0)
  {
    desired_heading += M_PI*2;
  }
  if (agent_heading < 0)
  {
    agent_heading += M_PI*2;
  }
  if ( desired_heading< agent_heading)
  {
    desired_heading += M_PI*2;
  }
  int lower_idx = (int)((desired_heading-agent_heading)/M_PI_2); //the quadrant in which the desired heading lies
  int upper_idx = lower_idx + 1;
  if (upper_idx == 4)
  {
    upper_idx = 0;
  }

  if (ranges[lower_idx] > ranges[upper_idx])
  {
    return true;
  }
  else
  {
    return false;
  }
}

// returns if going in a desired direction is safe
bool PSO::get_safe_direction(std::vector<float> ranges, float desired_heading, float threshold, float agent_heading)
{
  if( desired_heading < 0)
  {
    desired_heading += M_PI*2;
  }
  if (agent_heading < 0)
  {
    agent_heading += M_PI*2;
  }
  if ( desired_heading< agent_heading)
  {
    desired_heading += M_PI*2;
  }
  int lower_idx = (int)((desired_heading-agent_heading)/M_PI_2); //the quadrant in which the desired heading lies
  int upper_idx = lower_idx + 1;
  if (upper_idx == 4)
  {
    upper_idx = 0;
  }
 

  if (ranges[lower_idx] > threshold && ranges[upper_idx] > threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

laser_ray PSO::get_laser_reads(laser_ray ray, const uint16_t ID)
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
    ray.range = environment.env_diagonal;
    s.at(ID)->laser_ranges.push_back(environment.env_diagonal);
    std::vector<float> v = {laser_point.x,laser_point.y};
    s.at(ID)->laser_pnts.push_back(v);
  }

  return ray;
}