#include "particle.h"

void Particle::update_position()
{
	// NED frame
	// x+ towards North
	// y+ towards East

	cout << ID << " "  << controller.get_waitingtime(ID) << " ";
	state[0] += state[2]*dt; // position x
	state[1] += state[3]*dt; // position y

	controller.assess_situation(ID, situation);

	std::vector<float> q;
	q.assign(situation.begin(),situation.end());

	cout << ":  "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
				      << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7];

	// if (controller.get_waitingtime(ID) > 0)
	// 	std::transform (q.begin(), q.end(), q.begin(), std::bind1st(std::multiplies<float>(),1.0/((float)controller.get_waitingtime(ID)+1))); // take the average

	// cout << ID << ": "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
	// 			      << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7] << endl;

	state[2] = controller.get_velocity_command_radial(ID,0,situation); // velocity x
	state[3] = controller.get_velocity_command_radial(ID,1,situation); // velocity y

	cout << endl;

};