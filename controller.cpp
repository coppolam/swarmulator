#include "controller.h"
#include "auxiliary.h"
#include <cmath>
#include <fstream>
#include <mutex>
#include "agent.h"
#include "particle.h"
#include "main.h"
#include "randomgenerator.h"
#include <numeric> 
#include "omniscient_observer.h"
#include "parameters.h"
#include <unistd.h>

OmniscientObserver *o = new OmniscientObserver();

Controller::Controller(){};
Controller::~Controller(){};

float Controller::f_attraction(float u)
{
	// return 1/(1+exp(-5*(u-0.719*2))) + 1/(1+exp(-5*(u+0.719*2))) -1 ; //% sigmoid function -- long-range attraction
	return  1/(1+exp(-5*(u-0.8022))); //% sigmoid function -- long-range attraction
}


float Controller::f_attraction_bearing(float u, float b)
{
	// if ( b > (2*M_PI-0.5) || b < 0.5 || (b > (M_PI-0.5) && b < (M_PI+0.5) ))
		return 1/(1+exp(-5*(u-0.3502))) ; //% sigmoid function -- long-range attraction
	// else
		// return 1/(1+exp(-5*(u-0.8022))) ; //% sigmoid function -- long-range attraction
}

/*
	Repulsion function 
	TODO: Make parametrized
*/
float Controller::f_repulsion(float u)
{
	return -0.1/u; // basic function -- short-distance repulsion
}

/*
	Extra (Gaussian?) function 
	TODO: Make parametrized
*/
float Controller::f_extra(float u)
{
	return 0;
}

/*
	Get a velocity command along an axis based on knowledge of 
	position with respect to another agent.
*/
float Controller::get_attraction_velocity(float u, float b)
{
	float d;
	if (set)
	{
		d = saturate( f_attraction(u) + f_repulsion(u) + f_extra(u) ); 
		return d;
	}

	return 0;
}

void attractionmotion(const int &dim, const float &v_r, const float &v_b, float &v)
{	
	if (dim == 0)
		v = v_r * cos(v_b);
	else if (dim == 1)
		v = v_r * sin(v_b);
}

void latticemotion(const int &dim, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	// Back to Cartesian
	if (dim == 0)
		v += -v_adj * cos(bdes*2-v_b) ; // use for reciprocal alignment
	else if (dim == 1)
		v += -v_adj * sin(bdes*2-v_b) ; // use for reciprocal alignment
}

void circlemotion(const int &dim, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	if (dim == 0)
		v += v_adj * cos(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
	else if (dim == 1)
		v += v_adj * sin(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
}

vector<int> waiting(100,0);
vector<bool> circling(100,0);
vector<bool> hlvec(100,0);
vector<bool> qavg(100,0);

void Controller::filltemplate(vector<bool> &q, const float b_i, const float u, const float dmax)
{
	vector<float> blink;
			
	// Angles to check for neighboring links
	blink.push_back(0);
	blink.push_back(M_PI/4.0);
	blink.push_back(M_PI/2.0);
	blink.push_back(3*M_PI/4.0);
	blink.push_back(M_PI);
	blink.push_back(deg2rad(  180+45));
	blink.push_back(deg2rad(  180+90));
	blink.push_back(deg2rad(  180+135));
	blink.push_back(2*M_PI);

	// Determine link
	if (u < dmax)
	{
		for (int j = 0; j < (int)blink.size(); j++)
		{
			if ( abs(b_i - blink[j]) < deg2rad(22.49) ) 
			{
				if (j == (int)blink.size()-1) // last element is back to 0
					q[0] = true;
				else
					q[j] = true;
			}
		}
	}
}

int Controller::get_bearing_velocity(const vector<float> &bdes, const float v_b)
{

	vector<float> bv;
	for (int i = 0; i < (int)bdes.size()*5; i++)
	{
		for (int k = 0; k < (int)bdes.size(); i++)
		{
			bv.push_back(deg2rad( k));
		}
	}

	// Find what the desired angle is in bdes
	for (int i = 0; i < (int)bv.size(); i++)
	{
		if (i < 2)
			bv[i] = abs(bv[i]-2*M_PI-v_b);
		else if (i < 4)
			bv[i] = abs(bv[i]-M_PI-v_b);
		else if (i < 6)
			bv[i] = abs(bv[i]-v_b);
		else if (i < 8)
			bv[i] = abs(bv[i]+M_PI-v_b);
		else if (i < 10)
			bv[i] = abs(bv[i]+2*M_PI-v_b);
	}

    int minindex = 0;
    for(int i = 1; i < (int)bv.size(); i++)
    {
        if(bv[i] < bv[minindex])
            minindex = i;          
    }

    while (minindex >= (int)bdes.size()) // Reduce the index to 0 or 1
    	minindex -= (int)bdes.size();

    return minindex;
}

float Controller::get_velocity_command_radial(int ID, int dim)
{
	float u, v, b_i;
	v = 0;
	
	vector<float> bdes;
	// Desired angles, so as to create a matrix
	bdes.push_back(deg2rad(  0));
	bdes.push_back(deg2rad( 90));

	// Initialize some stuff
	float v_r   = 0.0;
	float v_b   = 0.0;
	float v_adj = 0.1;

	bool happy = false; // null assumption on happiness of the agent

	vector<bool> q(8,false); // = 8, the 8 positions we go around
	
	// Get vector of all neighbors from closest to furthest
	vector<int> closest = o->request_closest(ID);
	
	// For all neighbors detected (in simulation all agents) determine the 
	for (int i = 0; i < nagents-1; i++)
	{
		u   = o->request_distance(ID, closest[i]);
		b_i = o->request_bearing (ID, closest[i]);
		wrapTo2Pi(b_i);

		// For the number of knearest neighbors, get desirer radial and bearing attraction
		if (i < knearest)
		{
			// Uncomment this to simulate to simulate noise
			// v_b += wrapToPi_f(o->request_bearing(ID, closest[i]))+ getrand_float(-0.2, 0.2);
			// v_r += get_attraction_velocity(sqrt(u) + getrand_float(-0.1, 0.1),v_b);

			v_b += wrapToPi_f(o->request_bearing(ID, closest[i]));
			v_r += get_attraction_velocity(u,b_i);

		}
		filltemplate(q, b_i, u, 0.8);
	}
	int minindex = get_bearing_velocity(bdes, v_b);

	// cout << ID << ": "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
	// 			         << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7] << endl;

	vector<vector<bool>> links(4);
	links[0] = {0, 1, 1, 0, 0, 0, 0, 0};
	links[1] = {0, 0, 0, 0, 0, 0, 1, 1};
	links[2] = {0, 0, 0, 1, 1, 1, 0, 0};
	links[3] = {1, 0, 1, 0, 0, 0, 1, 0};

	// vector<vector<bool>> links(9);
	// links[0] = {0, 1, 1, 0, 0, 0, 0, 0};
	// links[1] = {0, 0, 0, 0, 0, 0, 1, 1};
	// links[2] = {0, 0, 0, 1, 1, 1, 0, 0};
	// links[3] = {1, 1, 1, 0, 0, 0, 1, 1};

	// links[4] = {0, 1, 1, 1, 1, 1, 0, 0};
	// links[5] = {0, 0, 0, 1, 1, 1, 1, 1};
	// links[6] = {1, 0, 1, 1, 1, 1, 1, 0};
	// links[7] = {1, 1, 1, 0, 0, 0, 1, 0};

	// links[8] = {1, 0, 1, 0, 0, 0, 1, 1};


	// Implement averaging method for q



	int hl=8;
	// Check if happy cycling through the links
	for (int i = 0; i < 4; i++)
	{
		// Quantify happiness level
		int s = 0;

		for (int j = 0; j < 8; j++){
			s+= q[j]^links[i][j]; // ^ = xor, it tells us if there is a match, so we measure the number of mistakes
			// On top of this, we could xor s with a 1 or a 0 so we can only measure errors in 1s and 0s rather than mistakes.
		}
		if (s < hl)
		{
			hl = s; // Score out set to the value of s if lower. We are looking for the minimum score. Can be changed to maximum
		}

		// Binary happy not happy
		if (q == links[i])
			happy = true;
	}

	// Did the situation improve?
	bool improved = false;
	if (hl < hlvec[ID])
	{	
		improved = true;
	}
	hlvec[ID] = hl;

	// Is the agent stuck?
	// if ( (q[0] && q[4]) || (q[1] && q[5]) || (q[2] && q[6]) || (q[3] && q[7]) )
	// 	happy = true;

	if (hl == 0 || happy )//|| alreadydone[ID]) // If happy, do what you gotta do
	{
		cout << ID << " happy " << endl ;
		// happyonce[ID] = true;
		// Action
		attractionmotion ( dim, v_r + v_adj , v_b, v);
		latticemotion    ( dim, v_adj , v_b, bdes[minindex], v);

		// Outflow
		circling[ID] = false;
		waiting[ID]  = 0;
	// 	if (waiting[ID] > 100)
	// 	{
	// 		circling[ID] = false;
	// 		waiting[ID]=0;
	// 	}
	// 	else
	// 		waiting[ID]++;
	// }
	}
	else if ( circling[ID] ) // In circling mode, circle around
	{
		attractionmotion ( dim,  v_r   , v_b,  v  );
		circlemotion     ( dim, -v_adj*hl , v_b,  bdes[minindex], v);

		float random = ((float) rand()) / (float) RAND_MAX;
		// cout << random << endl;

		if ( random > (float)1/(waiting[ID]))
		{
			circling[ID] = false;
			waiting[ID]=0;
		}
		else
			waiting[ID]++;
		// cout << ID << "circling" << endl;
		// // Action
		// attractionmotion ( dim,  v_r , v_b,  v  );
		// circlemotion     ( dim,  -v_adj , v_b,  bdes[minindex], v);

		// // // Outflow
		// // float random = ((float) rand()) / (float) RAND_MAX;
		// // if ( random > (float) hl / (waiting[ID]) )
		// if (improved)
		// {
		// 	cout << ID << " improved" << endl;
		// 	circling[ID] = false;
		// 	waiting[ID]  = 0;
		// }
		// else
		// {
		// 	waiting[ID]++;
		// }

	}
	else // In waiting mode
	{
		cout << ID << " waiting "<< waiting[ID]/simulation_updatefreq << endl;
		
		// Action
		attractionmotion ( dim, v_r , v_b, v);
		// latticemotion    ( dim, v_adj , v_b, bdes[minindex], v);
	float random = ((float) rand()) / (float) RAND_MAX;
		cout << random << endl;

		if ( 1*random > (float)1/waiting[ID])
		{
			circling[ID] = true;
			waiting[ID] = 0;
		}
		else
		{
			waiting[ID]++;
			cout << ID << " waiting "<< waiting[ID] << endl;
		}
		// // Outflow criteria to "circling behavior"
		// float random = ((float) rand()) / (float) RAND_MAX;
		// float k = 1;
		// if ( random > k*(8-hl) / (waiting[ID]/simulation_updatefreq + k*(8-hl)))
		// {
		// 	circling[ID] = true; // Switch to circling mode at next time-step
		// 	waiting [ID] = 0;    // Reset counter
		// }
		// else // Wait
		// {
		// 	waiting[ID]++; 		 // Increase counter
		// }

	}

	return v;

}


float Controller::get_velocity_command_cartesian(int ID, int dim)
{
	float v = 0.0;
	float u;
	int i = 0;

	#ifdef KNEAREST

	vector<int> closest = o->request_closest(ID);
	for (i = 0; i < knearest; i++)
	{
			u = o->request_distance_dim(ID, closest[i], dim);
			v += get_attraction_velocity(u,0);
	}

	#endif

	#ifdef FORCED
	
	std::ifstream infile("adjacencymatrix.txt");
	bool mat[nagents*nagents];
	while (i<(nagents*nagents))
	{
		infile >> mat[i];
		i++;
	}

	for (i = 0; i < nagents; i++)
	{
		if (i!=ID)
		{
			u = o->request_distance(ID, i, dim);
			v += (get_attraction_velocity(u,u) * mat[ID*nagents+i]);
		}
	}
	#endif

	#ifdef ROGUE
	if (ID == rogueID && (simulation_realtimefactor*simulation_time/1000000.0)>5.0)
	{
		v += 0.5;
	}
	#endif

	return v;

}

 
// If the saturation parameter is set, it will use it.
// Otherwise it has no effect
float Controller::saturate(float f)
{
	if (saturation) 
	{
		keepbounded(f,-saturation_limits,saturation_limits);
	}
	return f;
}

/*
	Set a saturation parameter on the controller object for the speed 
*/
void Controller::set_saturation(const float &lim)
{
	saturation = true;
	saturation_limits = lim;
}

void Controller::set_weights(const vector<float> &w)
{
	set = true;
	this->weights = w;
} 