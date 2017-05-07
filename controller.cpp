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

OmniscientObserver *o = new OmniscientObserver();

Controller::Controller(){};
Controller::~Controller(){};


float Controller::f_attraction(float u)
{
	return 1/(1+exp(-5*(u-0.7783))) + 1/(1+exp(-5*(u+0.7783))) -1 ; //% sigmoid function -- long-range attraction
}


float Controller::f_attraction_bearing(float u, float b)
{
	// if ( (b > (2*M_PI-0.5)) || (b < 0.5 ) || ((b > (M_PI-0.5)) && (b< (M_PI+0.5) ) ) )
	// 	return 1/(1+exp(-5*(u-0.4))) + 1/(1+exp(-5*(u+0.4))) -1 ; //% sigmoid function -- long-range attraction
	// else
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

void latticemotion(const int &dim, const float &v_b, const float &bdes, float &v)
{
	float v_adj = 0.2;

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

void circlemotion(const int &dim, const float &v_b, const float &bdes, float &v)
{
	float v_adj = 0.2;

	if (dim == 0)
	{
		    v += v_adj* cos(M_PI/2-v_b) ; // use for rotation
	}

	else if (dim == 1)
	{
			v += -v_adj * sin(M_PI/2-v_b) ; // use for rotation
	}
}

vector<bool> stuckonce(100);

float Controller::get_velocity_command_radial(int ID, int dim)
{
	float v_r   = 0.0;
	float v_b   = 0.0;

	vector<float> bdes;

	std::vector<float> bv;
	for (int i = 0; i < 5; i++)
	{
		if (i < 1)
		{
			bdes.push_back(deg2rad(  0));
			// bdes.push_back(deg2rad( 45));
			bdes.push_back(deg2rad( 90));
			// bdes.push_back(deg2rad(135));
		}
		bv.push_back(deg2rad(  0));
		// bv.push_back(deg2rad( 45));
		bv.push_back(deg2rad( 90));
		// bv.push_back(deg2rad(135));
	}
	int lbdes = bdes.size();

	vector<bool> q(lbdes*2,false);
	vector<bool> qs(lbdes*2,false);
	bool happy,stuck;

	float u, v, b_i;
	int i = 0;

	int cnt = 0;
	int nbs = 0;
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

		if (i < knearest)
		{
			v_b += wrapToPi_f(o->request_bearing(ID, closest[i])/knearest);// + getrand_float(-0.2, 0.2);
			v_r += get_individual_command(sqrt(u),v_b)/knearest;
		}
		b_i = o->request_bearing(ID, closest[i]);
		wrapTo2Pi(b_i);

		// calculate link type
		if (sqrt(u) < 1.4)
		{
			cnt++;
			for (int j = 0; j < lbdes*2; j++)
			{
				if (      j <lbdes  && ( abs(b_i - bdes[j]) < 0.7 ) )
				{
					q[j] = true;
					if (      j <lbdes  && ( abs(b_i - bdes[j]) < 0.1 ) )
						qs[j] = true;

					nbs++;
				}
				else if ( j>=lbdes && abs(b_i - (bdes[j-lbdes] + M_PI) ) < 0.7 )
				{				
					q[j] = true;
					if ( j>=lbdes && abs(b_i - (bdes[j-lbdes] + M_PI) ) < 0.1 )
						qs[j] = true;
					nbs++;
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
	bool superhappy = false;
	// #ifdef ROGUE
	// if (ID == rogueID)
	// {
	// int sum = std::accumulate(q.begin(), q.end(), 0);
	int sum=0;
	for (int i = 0; i < 4; i++)
	{
		sum +=q[i];
	}
	// cout << ID << " " << sum << endl;
	// if (ID == 3)
		cout << ID << ": "<< nbs << " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		if(simtime_seconds > 5.0 )// && simtime_seconds<50.0)
		{
			if (
				((  qs[0] &&  qs[1] && !qs[2] && !qs[3] )) || // not link 1
				(( !qs[0] && !qs[1] &&  qs[2] &&  qs[3] )) || // not link 2
				((  qs[0] && !qs[1] && !qs[2] &&  qs[3] )) || // not link 3
				(( !qs[0] &&  qs[1] &&  qs[2] && !qs[3] ))  // not link 4
				)
			{
				superhappy = true;
			}
			else if  (   // list here all the things that make happy with its position in the lattice.
				((  q[0] &&  q[1] && !q[2] && !q[3] )) || // not link 1
				(( !q[0] && !q[1] &&  q[2] &&  q[3] )) || // not link 2
				((  q[0] && !q[1] && !q[2] &&  q[3] )) || // not link 3
				(( !q[0] &&  q[1] &&  q[2] && !q[3] ))  // not link 4
			)	
			{
				happy = true;
				cout << ID << " happy " << endl;
			}
			else if (
				sum	 > 1 ||
				((  q[0] && !q[1] &&  q[2] && !q[3] ) || // not stuck 1
				 ( !q[0] &&  q[1] && !q[2] &&  q[3] ))    // not stuck 2)
				)
			{
				happy = false;
				stuck = true;
				cout << ID << " stuck " << endl;
			}
			else
{				happy = false;
				stuck = false;
				cout << ID << " unhappy " << endl;
}
		}

		if (!happy && !stuck )
			circlemotion  ( dim, v_b, bdes[minindex], v );
		
		else if ((stuck) || happy || stuckonce[ID])
			latticemotion ( dim, v_b, bdes[minindex], v ); //cout << ID << " happy " << endl;

		if (stuck)
			stuckonce[ID] = true;
		// if (!stuck)
		// 	stuckonce[ID] = false;
		
	// if (superhappyonce[ID] && happy)
	// 	return 0;
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

	for (i =/ 0; i < nagents; i++)
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