#include "particle.h"

void Particle::update_position()
{
	state[0] += state[2]*dt;
	state[1] += state[3]*dt;
	state[2] = controller.get_velocity_command_radial(ID,0);
	state[3] = controller.get_velocity_command_radial(ID,1);
};