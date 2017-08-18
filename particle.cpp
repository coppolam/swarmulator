#include "particle.h"

void Particle::update_position()
{
	// NED frame
	// x+ towards North
	// y+ towards East

	state[0] += state[2]*dt; // position x
	state[1] += state[3]*dt; // position y
	state[2] = controller.get_velocity_command_radial(ID,0); // velocity x
	state[3] = controller.get_velocity_command_radial(ID,1); // velocity y
};