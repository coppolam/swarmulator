#include "gas_seeking.h"
#include "draw.h"
#include "main.h"
#include "auxiliary.h"
#include <tuple>
#include "randomgenerator.h"
#include "mlp_inference.h"

void gas_seeking::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{

  random_generator rg;
  // load agent state
  std::vector<float> state = s.at(ID)->state;
  // load policy shape and params (weights/biasadd terms)
  std::vector<float> policy_params = load_vector(param->policy().c_str());
  std::vector<float> policy_shape = load_vector(param->policy_shape().c_str());

  // load laser ranger object with laser ranges
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
  }

  // initialize gas state with laser ranges
  std::vector<float> gas_state = s.at(ID)->laser_ranges;
  std::string euclidean = param->gas_euclidean(); // if True we feed distance to source [m] instead of gas concentration, for debugging purposes

  if(!strcmp(euclidean.c_str(), "False")) // if using gas concentration
  {
  //indexes in x-y grid, starting from bottom left corner
  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);

  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);
  float delta_gas = gas_conc - s.at(ID)->gas_read;
  gas_state.push_back(gas_conc/5000.); //normalizing, temporary solution
  gas_state.push_back(delta_gas);
  s.at(ID)->gas_read = gas_conc;
  }
  else
  {
    //compute euclidean distance to source from current agent position
    float eucl_dist = std::sqrt(pow((s.at(ID)->state[1]-environment.gas_obj.source_location[0]-environment.x_min),2)+pow((s.at(ID)->state[0]-environment.gas_obj.source_location[1]-environment.y_min),2));
    float delta_dist = 10*(s.at(ID)->gas_read - eucl_dist);
    gas_state.push_back(eucl_dist);
    gas_state.push_back(delta_dist);
    s.at(ID)->gas_read = eucl_dist;
  }
  // ** MLP INFERENCE ** //
  int action = float_inference(gas_state,policy_params,policy_shape);
  std::vector<float> ranges = s.at(ID)->laser_ranges;

  if(action == 0)
  { 
    v_x = 2.0;
    v_y = 0.0;
  }
  else if(action == 1)
  {
    v_x = 0.0;
    v_y = 1.0;
  }

  else if(action == 2)
  {
    v_x = 0.0;
    v_y = -1.0;
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
  // we didn't find anything (this should be rare if not impossible), we we return agent_pos and 0 distance
  else
  {
    s.at(ID)->laser_ranges.push_back(0);
    std::vector<float> v = {agent_pos.x,agent_pos.y};
    s.at(ID)->laser_pnts.push_back(v);
  }
}
