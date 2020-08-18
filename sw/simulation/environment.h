#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <stdint.h>
#include <vector>
#include <mutex>
#include "randomgenerator.h"



class Gasdata
{
  public:
    int num_it;
    size_t bmp_header_size;
    std::vector<float> source_location;
    std::vector<float> env_min;
    std::vector<float> env_max;
    std::vector<float> cell_sizes;
    std::vector<int> numcells;
    std::vector<std::vector<std::vector<int>>> gas_data;
    std::vector<int> max_gas;
};

class Environment
{
  

  
  random_generator rg;
public:
  /**
   * @brief class for storing all gas-related data of an environment
  */
  float x_min,x_max,y_min,y_max, env_size, env_diagonal;
  Gasdata gas_obj; //obj containing all gas information for this environment
  std::vector<std::vector<float>> food;
  std::vector<float> beacon;
  std::vector<std::vector<float>> free_points;
  std::vector<std::vector<float>> walls;
  float nest;
  /**
  * @brief Construct a new Environment object
  *
  */
  Environment();

  /**
   * @brief Destroy the Environment object
   *
   */
  ~Environment() {};

/**
 * @brief if param->gas_seeking is True, this loads gas data from txt files (concentrations + wind velocities)
*/
void load_gas_data(void);


/**
 * Gets x_min, x_max, y_min, y_max for a given env
*/

void get_min_max(std::vector<std::vector<float>> walls);
/**
 * @brief run the dungeon_generator.py if environment generation is "random"
*/
void generate_dungeon(void);

  /**
   * @brief Define the initial obstacle list according to the list in conf/environments/.txt
   * You can indicate obstacle list in the conf/parameters.xml file, under <environment>
   * Make sure the file exists!
   */
  void define_walls(void);
  
  /**
   * @brief generate a map with free space based on the walls loaded in the environment.
   */
  void complete_folder(void);

  /**
   * @brief Define the initial obstacle list according to the list in conf/environments/.txt
   * You can indicate obstacle list in the conf/parameters.xml file, under <environment>
   * Make sure the file exists!
   */
  void define_food(uint64_t n);


  void define_beacon(float x, float y);

  /**
   * @brief Returns a point within the environment.
   *
   * @return float
   */
  std::vector<float> start(void);

  /**
   * @brief Returns the furthers point from (0,0) in the environment, used for initialization so that
   * the robots can be initialized in the same spot.
   *
   * @return float
   */
  float limits(void);

  /**
   * @brief Add a new wall to the list, going from (x0,y0) to (x1,y1).
   * This is used to interactively create walls in the animation, just right click with the mouse!
   *
   * @param x0 Initial x
   * @param y0 Initial y
   * @param x1 Final x
   * @param y1 Final y
   */
  void add_wall(float x0, float y0, float x1, float y1);

  /**
   * Senses whether the next state will go through a wall
   *
   * @param ID Robot to consider
   * @param s_n Next state
   * @param s Current state
   * @return true if the lines intersect, meaning that it will go through a wall, so that we can handle it.
   * @return false if the lines do not intersect, so that the robot will not go through a wall and can act normally.
   */
  bool sensor(const uint16_t ID, std::vector<float> s_n, std::vector<float> s, float &angle);

  void grab_food(uint64_t food_ID);
  void drop_food();
  void eat_food(float);

  void loop(void);

  /**
   * Function used to draw all the walls in the animation. It is called by the animation thread.
   */
  void animate(void);
};

#endif /*ENVIRONMENT_H*/
