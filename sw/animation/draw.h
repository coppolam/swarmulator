#ifndef DRAW_H
#define DRAW_H

#include "glincludes.h"
#include <vector>
#include <sstream>      // std::stringstream
#include <mutex>

#include "particle.h"
#include "agent.h"
#include "main.h"
#include "drawingparams.h"

using namespace std;

/*
	Drawer object that takes care of OpenGL drawing for all elements of interest
	It is launched and managed by the animation thread.
*/

class draw {
public:

	// General functions
	void draw_triangle(double); // Draw a simple triangle
	void draw_circle(double r);
	void draw_axes();  	  // Draw global x y axes
	void draw_point();    // Draw a white point

	// Swarmulator specific functions
	void draw_data(); 	  // Draw relevant data in the corner
	void draw_axes_text(int dim);
	void draw_agent_number(int ID); // Draw an agent number in the location of the agent
	void draw_agent(int ID, float x, float y, float z); // Draw the actual agent
	void draw_centroid(float x, float y, float z); // Draw a dot at the swarm centroid

};

#endif /*DRAW_H*/