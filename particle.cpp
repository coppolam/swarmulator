#include "particle.h"

void Particle::update_position()
{
	// NED frame
	// x+ towards North
	// y+ towards East

	// cout << ID << " "  << controller.get_waitingtime(ID) << " ";
	state[0] += state[2]*dt + 0.5*state[4]*pow(dt,2); // position x
	state[1] += state[3]*dt + 0.5*state[5]*pow(dt,2); // position y

	controller.assess_situation(ID, situation);

	std::vector<float> q;
	q.assign(situation.begin(),situation.end());

	// cout << ":  "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
	// 			      << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7];

	// if (controller.get_waitingtime(ID) > 0)
	// 	std::transform (q.begin(), q.end(), q.begin(), std::bind1st(std::multiplies<float>(),1.0/((float)controller.get_waitingtime(ID)+1))); // take the average

	// cout << ID << ": "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
	// 			      << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7] << endl;

	state[2] = state[2] + state[4]*dt; // velocity x
	state[3] = state[3] + state[5]*dt; // velocity y
	
	state[4] = -2*( state[2] - controller.get_velocity_command_radial(ID,0,situation) ) ;
	state[5] = -2*( state[3] - controller.get_velocity_command_radial(ID,1,situation) ) ;

	// cout << endl;

};