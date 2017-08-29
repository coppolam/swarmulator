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
#include <random>

OmniscientObserver *o = new OmniscientObserver();

Controller::Controller(): waiting(100,0){
		srand(time(NULL)); // Seed the time
};
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

int bool2int(vector<bool> t)
{
	int n = 0; //initialize

	// cout << " string:[";
	for (int i = 0; i < 8; i++)
	{
		// cout << t[i];
		n += (int)t[i]*(int)pow(2,7-i);
	}   	
	// cout << "]";
	return n;

}


void attractionmotion(const int &dim, const float &v_r, const float &v_b, float &v)
{	
	if (dim == 0)
		v = v_r * cos(v_b);
	else if (dim == 1)
		v = v_r * sin(v_b);
}

void latticemotion(const int &dim, const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	attractionmotion ( dim, v_r + v_adj, v_b, v);

	// Back to Cartesian
	if (dim == 0)
		v += -v_adj * cos(bdes*2-v_b) ; // use for reciprocal alignment
	else if (dim == 1)
		v += -v_adj * sin(bdes*2-v_b) ; // use for reciprocal alignment
}

void circlemotion(const int &dim, const float &v_r, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	attractionmotion ( dim, v_r, v_b, v );

	if (dim == 0)
		v += v_adj * cos(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
	else if (dim == 1)
		v += v_adj * sin(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
}


void Controller::fill_template(vector<float> &q, const float b_i, const float u, float dmax)
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
					q[0] = 1;
				else
					q[j] = 1;
			}
		}
	}
}

int Controller::get_bearing_velocity(const vector<float> &bdes, const float v_b)
{

	vector<float> bv;
	for (int i = 0; i < 5; i++)
	{
		bv.push_back( deg2rad(0));
		bv.push_back( deg2rad(90));
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

void Controller::assess_situation(int ID, vector<float> &q_old)
{	
	// Get new q
	vector<float> q(8,0);
	float u, b_i;
	vector<int> closest = o->request_closest(ID); // Get vector of all neighbors from closest to furthest
	for (int i = 0; i < nagents-1; i++)
	{
		u   = o->request_distance(ID, closest[i]);
		b_i = o->request_bearing (ID, closest[i]);
		wrapTo2Pi(b_i);
		fill_template(q, b_i, u, sqrt(pow(0.7,2)+pow(0.7,2)));
	}

	// if (waiting[ID] < 1)
	// {	
	// 	q_old.assign(q.begin(),q.end());
	// }
	// else
	// {
		std::transform (q.begin(), q.end(), q_old.begin(), q_old.begin(), std::plus<float>()); // sum
		for (int i = 0; i < 8; i++)
		{
			if (q[i] == 0)
				q_old[i] = 0;
		}
	// }
}

vector<bool> circling(100,0);
vector<int> hlvec(100,0);
vector<int> tracenumber(100,0);
vector<int> tracking(100,0);

float Controller::get_velocity_command_radial(int ID, int dim, vector<float> q)
{
	float u, v, b_i;
	v = 0;
	
	// Desired angles, so as to create a matrix
	vector<float> bdes;
	bdes.push_back(deg2rad(  0));
	bdes.push_back(deg2rad( 90));

	// Initialize some stuff
	float v_r   = 0.0;
	float v_b   = 0.0;
	float v_adj = 0.1;

	bool happy = false; // null assumption on happiness of the agent

	vector<int> closest = o->request_closest_inrange(ID,sqrt(pow(0.7,2)+pow(0.7,2))); // Get vector of all neighbors from closest to furthest
	
	// For all neighbors detected (in simulation all agents) determine the 
	v_r += get_attraction_velocity(o->request_distance(ID, closest[0]), wrapTo2Pi_f(o->request_bearing (ID, closest[0])));
	v_b += wrapToPi_f(o->request_bearing(ID, closest[0]));
	// Uncomment this to simulate to simulate noise
	// v_b += wrapToPi_f(o->request_bearing(ID, closest[0]))+ getrand_float(-0.2, 0.2);
	// v_r += get_attraction_velocity(u + getrand_float(-0.1, 0.1),v_b);

	int minindex = get_bearing_velocity(bdes, v_b);
	// vector<vector<bool>> links(4);
	// links[0] = {0, 1, 1, 0, 0, 0, 0, 0};
	// links[1] = {0, 0, 0, 0, 0, 0, 1, 1};
	// links[2] = {0, 0, 0, 1, 1, 1, 0, 0};
	// links[3] = {1, 0, 1, 0, 0, 0, 1, 0};
					  // 0  1  2  3
			 // trace = {0, 3, 1, 2};
	//	--------------------------------------------------
	// vector<int> trace = {0, 2, 3, 1};
// 
	vector<vector<bool>> links(9);
	links[0] = {0, 1, 1, 0, 0, 0, 0, 0};
	links[1] = {0, 0, 0, 0, 0, 0, 1, 1};
	links[2] = {0, 0, 0, 1, 1, 1, 0, 0};
	links[3] = {1, 1, 1, 0, 0, 0, 1, 1};
	links[4] = {0, 1, 1, 1, 1, 1, 0, 0};
	links[5] = {0, 0, 0, 1, 1, 1, 1, 1};
	links[6] = {1, 0, 1, 1, 1, 1, 1, 0};
	links[7] = {1, 1, 1, 0, 0, 0, 1, 0};
	links[8] = {1, 0, 1, 0, 0, 0, 1, 1};

	// 	                    0  1  2  3  4  5  6  7  8
	//            trace =  {0, 7, 3, 8, 1, 5, 6, 4, 2};
	// vector<int> trace = {0, 4, 8, 2, 7, 5, 6, 1, 3};

	// 	                    0  1  2  3  4  5  6  7  8
	//            trace =  {1, 5, 8, 2, 6, 3, 4, 7, 0};
	// --------------------------------------------------
	vector<int> trace = {8, 0, 3, 5, 6, 1, 4, 7, 2};

	
	int hl = 0;
	int th = 50;
	vector<bool> t(8,0); //LINK holder with false assumption

	// Check if happy cycling through the links
	for (int i = 0; i < (int)links.size(); i++)
	{
		// Quantify happiness level
		int s = 0;

		for (int j = 0; j < 8; j++)
		{ 
			// Use classifying threshold and write it to Link holder
			if (q[j] > th)
				t[j] = true;

			// (XOR)
			s+= ( t[j]^links[i][j] ); // ^ = XOR, it tells us if there is a match, so we measure the number of "mistakes"
		}

		if ( (8-s) > hl )
		{
			tracenumber[ID] = trace[i]; // save this as the most similar link
			hl = 8-s; // Score out set to the value of s if lower. We are looking for the minimum score. Can be changed to maximum
		}

		// Binary happy not happy
		if (hl == 8)
		{
			happy = true;
		}
	}

	if ( !happy && ( (q[0]>th && q[4]>th) || (q[1]>th && q[5]>th) || (q[2]>th && q[6]>th) || (q[3]>th && q[7]>th) ))
	{
		happy = true;
		tracenumber[ID] = 0;
		hl = 8;
	}

	// vector <int> action   = {1,5,5,5,1,5,5,4,1,4,4,4,1,4,4,5,1,1,1,3,1,5,5,3,1,3,3,3,1,3,3,5,5,1,1,1,1,1,5,6,6,1,1,7,1,1,1,5,5,1,5,5,5,1,5,6,6,1,1,7,1,1,1,1,1,5,5,1,5,5,5,1,4,4,4,1,4,4,4,1,5,1,5,1,1,5,5,1,6,1,3,1,1,1,3,5,5,1,1,1,5,5,5,6,6,1,1,7,4,1,1,5,5,5,5,5,5,5,5,6,6,1,1,7,7,1,1,2,2,2,2,2,2,5,5,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,5,5,2,2,8,2,5,5,6,6,2,2,8,2,2,2,5,5,5,5,5,5,5,5,6,6,2,2,8,2,2,2,7,3,5,5,5,5,5,5,4,4,4,4,4,4,4,4,5,5,5,5,8,5,5,5,6,6,6,6,8,3,8,3,5,5,5,5,8,5,5,5,6,6,4,4,8,4,4,4,5,5,5,5,5,5,5,5,6,6,6,6,8,7,8,1};
	// vector <int> maxscore = {7,7,7,7,7,8,8,7,7,7,7,7,7,7,7,7,7,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,6,6,6,7,7,7,7,7,6,7,6,7,6,8,8,7,7,7,7,7,7,7,7,7,6,7,6,7,6,7,6,7,7,7,7,8,8,7,7,7,7,7,7,7,7,7,7,6,6,7,6,7,7,7,7,6,6,7,6,6,6,7,7,6,6,6,6,7,7,7,7,6,6,7,6,6,6,8,8,7,7,7,7,7,7,7,7,6,6,7,6,6,6,7,7,7,7,7,7,8,8,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,6,6,7,6,7,7,7,7,6,6,7,6,6,6,8,8,7,7,7,7,7,7,7,7,6,6,7,6,6,6,7,6,7,7,7,7,8,8,7,7,7,7,7,7,7,7,7,7,6,6,7,6,7,7,7,7,6,6,7,5,6,5,7,7,6,6,7,6,7,7,7,7,6,6,7,6,6,6,8,8,7,7,7,7,7,7,7,7,6,6,7,5,6,0};
	// vector <int> ppx = {0,1,1, 1, 0,-1,-1,-1};
	// vector <int> ppy = {1,1,0,-1,-1,-1, 0, 1};
	// bool tried  = false;
	// Did the situation improve?
	bool improved = false;
	if (hl > hlvec[ID]){
			improved = true;
			if (waiting[ID] > 200)
			waiting[ID] = 0;
			}
	
	if ( happy )// If happy, do what you gotta do
	{
		// if (dim == 1)
			// cout << " \t happy";
		tried = false;
		circling[ID] = false; // flag you are not circling
		hlvec[ID] = hl;

		if ( !circling[closest[0]] )
			latticemotion    ( dim, v_r, v_adj , v_b, bdes[minindex], v);
		else
			attractionmotion ( dim, v_r, v_b, v);

	}
	else if ( circling[ID] ) // In circling mode, circle around
	{
		// if (dim == 1)
			// cout << " \t circling " << hlvec[ID] << " " << hl;

		// if (!circling[closest[0]])
		circlemotion     ( dim,  v_r, v_adj , v_b,  bdes[minindex], v );

		// if (hl < maxscore[bool2int(t)-1])
		// {
		// 	int	actid = action[bool2int(t)-1];
		// 	cout << "ID " << ID << ": " << bool2int(t) << " hs:" << hl << " max:" << maxscore[bool2int(t)-1] << " act " << ppx[actid-1] << " " << ppy[actid-1] << endl;
		// 	if (dim==0)
		// 	{	v = ppy[actid-1]*v_adj;
		// 		tried = true;
		// 	}
		// 	else if (dim==1){
		// 		tried = true;
		// 		v = ppx[actid-1]*v_adj;
		// 	}
		// }

		if ( (improved && waiting[ID]>200) || waiting[ID] > 1000 )
		{
			circling[ID] = false; // stop circling
			waiting [ID] = 0; // reset counter
		}

		else
		{
			if (dim == 0) // increase counter
				waiting[ID]++;
		}
	}
	else // In waiting mode
	{	
		// if (dim == 1)
			// cout << "\t waiting "<< hlvec[ID] << " " << hl;

		if ( !circling[closest[0]] )
			latticemotion    ( dim, v_r, v_adj , v_b, bdes[minindex], v);
		else
			attractionmotion ( dim, v_r, v_b, v);

		int finalNum = 200;
		if ( (float)waiting[ID]/pow((float)(links.size() - tracenumber[ID]),1) > finalNum )
		{
			circling[ID] = true; // start circling
			waiting [ID] = 0; // reset counter 
		}
		else
		{
			if (dim == 0)	// increase counter
				waiting[ID]++;
		}

	}
		
	hlvec[ID] = hl;

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

int Controller::get_waitingtime(int ID)
{
	return waiting[ID];
} 