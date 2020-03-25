#ifndef DRAW_H
#define DRAW_H

#include <GL/freeglut.h>
#include <vector>
#include <sstream>      // std::stringstream
#include <mutex>

#include "main.h"
#include "drawingparams.h"
#include "settings.h"
#include "includes_agents.h"

using namespace std;

/**
 * @brief Object that takes care of OpenGL drawing for all elements of interest.
 * This class contains general openGL function that
 * can be used to animate and visualize the swarm at runtime.
 * It is launched and managed by the animation thread.
 */
class draw
{
public:
  /******* General openGL functions ********/

  /**
   * Draw a simple triangle of size scl
   *
   * @param s Scale of the triangle
   */
  void draw_triangle(double s);

  /**
   * Draw a red circle of radius r
   *
   * @param r Radius of the circle
   */
  void draw_circle(double r);

  /**
   * Draw a white unfilled circular loop
   *
   * @param r Radius of the circular loop
   */
  void draw_circle_loop(double r);

  /**
   * Draw a white line from (0,0) to (x,y)
   *
   * @param x x position (East)
   * @param y y position (North)
   */
  void draw_line(float x, float y);

  /**
   * Draw the global x and y axes at (0,0)
   */
  void draw_axes();

  /**
   * Draw a line (used for walls)
   */
  void draw_line(float x0, float y0, float x1, float y1);

  /**
  * Draw a small white point
  */
  void draw_point();

  /******* Swarmulator higher level functions ********/

  /**
   * Draw relevant simulation data in the bottom left corner (like the time of simulation)
   */
  void draw_data();

  /**
   * Write the x,y label on the global axis along the given dimension dim
   *
   * @param dim Specifies (=0) for writing the x label and (=1) for writing the y label
   */
  void draw_axes_text(uint8_t dim);

  /**
   * Draw the ID number of an agent on top of the agent
   *
   * @param ID The ID of the robot in question
   */
  void draw_agent_number(uint8_t ID);

  /**
   * Draw the agent (uses internal function defined by the agent class)
   *
   * @param ID The ID of the robot in question
   * @param x The x position of the robot
   * @param y The y position of the robot
   * @param z The z position of the robot
   */
  void draw_agent(uint8_t ID, float x, float y, float z);

  /**
   * Draw a line showing the velocity of the agent
   *
   * @param ID The ID of the robot in question
   * @param x The x position of the robot (East)
   * @param y The y position of the robot (North)
   * @param z The z position of the robot
   * @param v_x The velocity of the robot in x (global)
   * @param v_y The velocity of the robot in y (global)
   */
  void draw_velocity_arrow(uint8_t ID, float x, float y, float z, float v_x, float v_y); // Draw velocity arrow

  /**
   * Draw a dot at the swarm centroid position (x,y,z)
   *
   * @param x The x position of the centroid
   * @param y The y position of the centroid
   * @param z The z position of the centroid
   */
  void draw_centroid(float x, float y, float z); //
};

#endif /*DRAW_H*/
