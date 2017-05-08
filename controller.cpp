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
	return 1/(1+exp(-5*(u-0.719))) + 1/(1+exp(-5*(u+0.719))) -1 ; //% sigmoid function -- long-range attraction
}


float Controller::f_attraction_bearing(float u, float b)
{
	if ( b > (2*M_PI-0.5) || b < 0.5 || (b > (M_PI-0.5) && b < (M_PI+0.5) ))
		return 1/(1+exp(-5*(u-0.4))) + 1/(1+exp(-5*(u+0.4))) -1 ; //% sigmoid function -- long-range attraction
	else
		return 1/(1+exp(-5*(u-1.3))) + 1/(1+exp(-5*(u+1.3))) -1 ; //% sigmoid function -- long-range attraction
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
float Controller::get_individual_command(float u, float b)
{
	float d;
	if (set)
	{
		d = saturate( f_attraction_bearing(u,b) + f_repulsion(u) + f_extra(u) ); 
		return d;
	}

	return 0;
}

void attractionmotion(const int &dim, const float &v_r, const float &v_b, float &v)
{	
	if (dim == 0)
	{
		v = v_r * cos(v_b);
	}

	else if (dim == 1)
	{
		v = v_r * sin(v_b);
	}
}

void latticemotion(const int &dim, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	// Back to Cartesian
	if (dim == 0)
	{
		v += -v_adj * sin(bdes*2+M_PI/2-v_b) ; // use for reciprocal alignment
	}
	else if (dim == 1)
	{	
		v += v_adj * cos(bdes*2+M_PI/2-v_b) ; // use for reciprocal alignment
	}
}

void circlemotion(const int &dim, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	if (dim == 0)
	{
		v += v_adj * cos(M_PI/2-v_b); // use for rotation
	}

	else if (dim == 1)
	{
		v += -v_adj * sin(M_PI/2-v_b); // use for rotation
	}
}

vector<int> mode(100,0);

float Controller::get_velocity_command_radial(int ID, int dim)
{
	float v_r   = 0.0;
	float v_b   = 0.0;
	float v_adj = 0.2;

	vector<float> bdes;
	vector<float> bv;
	vector<float> blink;

	for (int i = 0; i < 5; i++)
	{
		if (i < 1)
		{
			bdes.push_back(deg2rad(  0));
			// bdes.push_back(deg2rad( 45));
			bdes.push_back(deg2rad( 90));
			// bdes.push_back(deg2rad(135));

			blink.push_back(deg2rad(  0));
			// blink.push_back(deg2rad(  45));
			blink.push_back(deg2rad(  90));
			// blink.push_back(deg2rad(  135));
			blink.push_back(deg2rad(  180));
			// blink.push_back(deg2rad(  225));
			blink.push_back(deg2rad(  270));
			// blink.push_back(deg2rad(  315));

		}
		bv.push_back(deg2rad(  0));
		// bv.push_back(deg2rad( 45));
		bv.push_back(deg2rad( 90));
		// bv.push_back(deg2rad(135));
	}
	int lbdes = bdes.size();

	vector<bool> q(lbdes*2,false);
	bool happy = false;
	bool stuck = false;

	float u, v, b_i;
	int i = 0;

	int cnt = 0;
	vector<int> closest = o->request_closest(ID);

	for (i = 0; i < nagents-1; i++)
	{
		// Normal attraction
		u = 0;
		for (int d = 0; d < 2; d++) // Distance to closest
		{
			float dd = o->request_distance(ID, closest[i], d);
			u += pow(dd,2);
		}

		b_i = o->request_bearing(ID, closest[i]);
		wrapTo2Pi(b_i);

		if (i < knearest)
		{
			// v_b += wrapToPi_f(o->request_bearing(ID, closest[i]))+ getrand_float(-0.2, 0.2);
			// v_r += get_individual_command(sqrt(u) + getrand_float(-0.2, 0.2),v_b);

			v_b += wrapToPi_f(o->request_bearing(ID, closest[i]));
			v_r += get_individual_command(sqrt(u),b_i);

			if (ID == 3)
				cout << b_i << endl;
		}


		// calculate link type
		if (sqrt(u) < 0.8)
		{
			cnt++;
	
			for (int j = 0; j < lbdes*2; j++)
			{
				if ( abs(b_i - blink[j]) < 0.4 ) 
				{
					q[j] = true;
				}
			}
		}

	}

	for (int i = 0; i < lbdes*5; i++)
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
    for(int i = 1; i < lbdes*5; i++)
    {
        if(bv[i] < bv[minindex])
            minindex = i;          
    }

    while (minindex >= lbdes)
    	minindex -= lbdes;

	attractionmotion (dim,v_r,v_b,v);
	// int sum=0;
	// for (int i = 0; i < 8; i++)
	// {
	// 	sum +=q[i];
	// }

	// 		if  (   // list here all the things that make happy with its position in the lattice.
	// 			((  q[0] &&  q[1] && !q[2] && !q[3] )) || // link 1
	// 			(( !q[0] && !q[1] &&  q[2] &&  q[3] )) || // link 2
	// 			((  q[0] && !q[1] && !q[2] &&  q[3] )) || // link 3
	// 			(( !q[0] &&  q[1] &&  q[2] && !q[3] ))    // link 4
	// 		)	
	// 		{
	// 			happy = true;
	// 			cout << ID << " happy " << mode[ID] <<endl;
	// 		}
	// 		else if (sum >2 ||
	// 				 (  q[0] && q[2] ) || // stuck 1
	// 			     (  q[1] && q[3] ) )
	// 		{
	// 			stuck = true;
	// 			cout << ID << " stuck " << mode[ID] << endl;	
	// 		}
	// 		else
	// 		{
	// 			cout << ID << " unhappy " << mode[ID] << endl;
	// 		}

			// if ( !happy && !stuck && ( mode[closest[0]] == 0))
			// {
			// 	circlemotion  ( dim, 0.4, v_b, bdes[minindex], v);
			// }
			// else if ( mode[closest[0]] == 0)
			// {
				latticemotion ( dim, v_adj, v_b, bdes[minindex], v);
			// }
			// else {
			// 	mode[ID] = 0;
			// }


		// 	// if (simtime_seconds < 10 ){
		// 		latticemotion ( dim, v_b, bdes[minindex], v);
		// 		mode[ID] = 1;
		// 	}
		// 	else
		// 	{
		// 	if (mode[closest[0]] != 1)
		// 	{
		// 		circlemotion  ( dim, v_b, bdes[minindex], v);
		// 	}
		// }
				
// 			if (simtime_seconds > 60 && simtime_seconds < 140)
// {
// 			if ( !happy && !stuck )
// 				circlemotion  ( dim, v_b, bdes[minindex], v);

// 			else if ( !happy || stuck )
// 				latticemotion ( dim, v_b, bdes[minindex], v);
// }
// else

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
			u = o->request_distance(ID, closest[i], dim);
			v += get_individual_command(u,u);
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
			v += (get_individual_command(u,u) * mat[ID*nagents+i]);
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
	// TODO: FIX
	// if (saturation) 
	// {
	// 	keepbounded(f,-saturation_limits,saturation_limits);
	// }
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