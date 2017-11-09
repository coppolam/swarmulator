#include "particle.h"

void Particle::update_position()
{
	// NED frame
	// x+ towards North
	// y+ towards East

	// Position
	state[0] += state[2]*dt + 0.5*state[4]*pow(dt,2); // position x
	state[1] += state[3]*dt + 0.5*state[5]*pow(dt,2); // position y

	float v_x, v_y;
	controller.get_velocity_command(ID, v_x, v_y);
	
	// Velocity
	state[2] += state[4]*dt; // velocity x
	state[3] += state[5]*dt; // velocity y

	// Acceleration
	state[4] = -2*( state[2] - v_x ) ; // Acceleration x 
	state[5] = -2*( state[3] - v_y ) ; // Acceleration y
	
};