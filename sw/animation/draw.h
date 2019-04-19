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
 * Object that takes care of OpenGL drawing for all elements of interest
 * It is launched and managed by the animation thread.
 */
class draw
{
public:
  /******* General openGL functions ********/

  /**
   * Draw a simple triangle of size scl
   */
  void draw_triangle(double s);

  /**
   * Draw a red circle of radius r
   */
  void draw_circle(double r);

  /**
   * Draw a white unfilled circle
   */
  void draw_circle_loop(double d);

  /**
   * Draw a white line from (0,0) to (x,y)
   */
  void draw_line(float x, float y);

  /**
   * Draw the global x and y axes at (0,0)
   */
  void draw_axes();

  /**
   * Draw arena walls
   */
  void draw_walls();

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
   */
  void draw_axes_text(uint8_t dim);

  /**
   * Draw the ID number of an agent on top of the agent
   */
  void draw_agent_number(uint8_t ID);

  /**
   * Draw the agent (uses internal function defined by the agent class)
   */
  void draw_agent(uint8_t ID, float x, float y, float z);

  /**
   * Draw a line showing the velocity of the agent
   */
  void draw_velocity_arrow(uint8_t ID, float x, float y, float z, float v_x, float v_y); // Draw velocity arrow

  /**
   * Draw a dot at the swarm centroid position (x,y,z)
   */
  void draw_centroid(float x, float y, float z); //
};

#endif /*DRAW_H*/
