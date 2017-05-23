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
	// if ( b > (2*M_PI-0.5) || b < 0.5 || (b > (M_PI-0.5) && b < (M_PI+0.5) ))
	// 	return 1/(1+exp(-5*(u-0.4))) + 1/(1+exp(-5*(u+0.4))) -1 ; //% sigmoid function -- long-range attraction
	// else
		return 1/(1+exp(-5*(u-1.4391
))) + 1/(1+exp(-5*(u+1.4391
))) -1 ; //% sigmoid function -- long-range attraction
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
		v += -v_adj * cos(bdes*2-v_b) ; // use for reciprocal alignment
	}
	else if (dim == 1)
	{	
		v += -v_adj * sin(bdes*2-v_b) ; // use for reciprocal alignment
	}

}

void circlemotion(const int &dim, const float &v_adj, const float &v_b, const float &bdes, float &v)
{
	if (dim == 0)
	{
		v += v_adj * cos(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
	}

	else if (dim == 1)
	{
		v += v_adj * sin(v_b-M_PI/2); // use for rotation (- clockwise, + anti-clockwise )
	}
}

vector<int> waiting(100,0);
vector<bool> circling(100,0);
vector<bool> masterlink(100,0);

vector<bool> alreadydone(100,0);
vector<float> bstore(100,0);
vector<int> cstore(100,0);
vector<int> neg(100,0);

float Controller::get_velocity_command_radial(int ID, int dim)
{
	float v_r   = 0.0;
	float v_b   = 0.0;
	float v_adj = 0.1;

	vector<float> bdes;
	vector<float> bv;
	vector<float> blink;

	for (int i = 0; i < 5; i++)
	{
		if (i < 1)
		{
			bdes.push_back(deg2rad(  0));
			bdes.push_back(deg2rad( 90));

			blink.push_back(deg2rad(  0));
			blink.push_back(deg2rad(  45));
			blink.push_back(deg2rad(  90));
			blink.push_back(deg2rad(  135));
			blink.push_back(deg2rad(  180));
			blink.push_back(deg2rad(  180+45));
			blink.push_back(deg2rad(  180+90));
			blink.push_back(deg2rad(  180+135));
		}

		bv.push_back(deg2rad(  0));
		bv.push_back(deg2rad( 90));

	}
	int lbdes = bdes.size();

	vector<bool> q(lbdes*4,false);
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

		}


		// calculate link type
		if (sqrt(u) < 1.5)
		{
			cnt++;
	
			for (int j = 0; j < lbdes*4; j++)
			{
				if ( abs(b_i - blink[j]) < deg2rad(20) ) 
				{
					q[j] = true;
				}
			}
		}

	}

	// // Find angle of interest
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

	// cout << ID << ": "<< " "  << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
	// 			      << " "  << q[4] << " " << q[5] << " " << q[6] << " " << q[7] << endl;

	if  (   // list here all the things that make happy with its position in the lattice.
		// ((  q[0] &&  q[1] && !q[2] && !q[3] )) || // link 1
		// (( !q[0] && !q[1] &&  q[2] &&  q[3] )) || // link 2
		// ((  q[0] && !q[1] && !q[2] &&  q[3] )) || // link 3
		// (( !q[0] &&  q[1] &&  q[2] && !q[3] ))    // link 4
		// (( !q[0] &&  q[1] &&  q[2] && !q[3] )) || // link 1
		// (( !q[0] && !q[1] && !q[2] &&  q[3] )) || // link 2
		// ((  q[0] && !q[1] && !q[2] && !q[3] ))

		// Triangle
		((  q[0] &&  q[1] && !q[2] && !q[3] && !q[4] && !q[5] && !q[6] && !q[7])) || // not link 1
		(( !q[0] && !q[1] && !q[2] && !q[3] && !q[4] && !q[5] &&  q[6] &&  q[7])) || // not link 2
		(( !q[0] && !q[1] && !q[2] &&  q[3] &&  q[4] &&  q[5] && !q[6] && !q[7])) 
		// ||   
		// (( !q[0] && !q[1] && !q[2] &&  q[3] &&  q[4] &&  q[5] &&  q[6] &&  q[7])) || // not link 3
		// (( !q[0] &&  q[1] &&  q[2] &&  q[3] &&  q[4] &&  q[5] && !q[6] && !q[7])) || // not link 3
		// ((  q[0] && !q[1] &&  q[2] &&  q[3] &&  q[4] &&  q[5] &&  q[6] && !q[7])) || // not link 3
		// ((  q[0] && !q[1] && !q[2] &&  q[3] && !q[4] && !q[5] &&  q[6] && !q[7]))    // not link 3

		// M
		// (( q[0] && !q[1] && !q[2] && !q[3] && !q[4] && !q[5] &&  q[6] && !q[7])) || // not link 1 
		// (( q[0] && !q[1] &&  q[2] && !q[3] && !q[4] && !q[5] && !q[6] && !q[7])) || // not link 1 
		// ((!q[0] && !q[1] && !q[2] && !q[3] &&  q[4] &&  q[5] && !q[6] && !q[7])) ||
		// ((!q[0] && !q[1] && !q[2] &&  q[3] &&  q[4] && !q[5] && !q[6] && !q[7])) ||
		// ((!q[0] &&  q[1] &&  q[2] && !q[3] && !q[4] && !q[5] &&  q[6] &&  q[7]))
	)	
	{
		happy = true;
		// cout << ID << " HAPPY" << endl;
	}

	else if ( 
		    // (  q[0] && q[2] ) || // stuck 1
		    // (  q[1] && q[3] ) )
		     (  q[0] && q[4] ) || // stuck 1
		     (  q[1] && q[5] ) ||
		     (  q[2] && q[6] ) || // stuck 1
		     (  q[3] && q[7] ) )
	{
		stuck = true;
		// cout << ID << " STUCK" << endl;
	}

	if ((!happy && !stuck) &&  !q[0] )// && !q[1] && !q[2]) ))
	{
		if (dim == 0){waiting[ID]++;}
	}
	else
	{
		waiting[ID] = 0;
	}

	int waittime = 30;
	int tw = waittime*simulation_updatefreq;

	if ( !circling[closest[0]] && (!happy && !stuck) && ((!q[0]) ) && waiting[ID] >= tw  )
	{
		circling[ID] = true; // !lattice && !static
		
		if( ( (wrapTo2Pi_f(v_b) <= wrapTo2Pi_f(bstore[ID]+M_PI/2)) && closest[0] == cstore[ID]) || waiting[ID] > tw*3 )
		{
			attractionmotion ( dim, v_r   , v_b,  v                );
			circlemotion     ( dim,  v_adj , v_b,  bdes[minindex], v);

		if (waiting[ID] > tw*3+400)
				waiting[ID] = tw+30;
		}
		
		else
		{			
			attractionmotion ( dim, v_r + v_adj , v_b, v);
			latticemotion    ( dim, v_adj      , v_b, bdes[minindex], v);
		}
		
		if (cstore[ID] != closest[0] && alreadydone[ID])
		{
			alreadydone[ID] = true;
			bstore[ID] = wrapTo2Pi_f(v_b);
			cstore[ID] = closest[0];
		}

		cout << ID << " " << neg[ID] << endl;//waiting[ID] <<" " << cstore[ID] << " " << wrapTo2Pi_f(bstore[ID]+M_PI/2) << " " << wrapTo2Pi_f(v_b) << endl;
	}


	// else if ( !circling[closest[0]] && 
	// 	(
	// 	((  q[0] &&  !q[2] &&  q[3] && !q[4] && !q[5] && !q[6]  )) ||   // not link 3
	// 	((  q[0] &&  !q[2] && !q[3] && !q[4] &&  q[5] && !q[6]  ))    // not link 3
	// 	)
	// 	)
	// {
	// 	if (dim == 0)
	// 		v += -0.1;
	// 			alreadydone[ID] = false;

	// }


	// else if ( !circling[closest[0]] && 
	// 	(
	// 	(( !q[0] &&  q[2] &&  !q[4] && !q[5] && !q[6] &&  q[7])) ||   // not link 3
	// 	(( !q[0] &&  q[2] &&  !q[4] &&  q[5] && !q[6] && !q[7]))    // not link 3
	// 	)
	// 	)
	// {
	// 	if (dim == 1)
	// 		v += -0.1;
	// 			alreadydone[ID] = false;

	// }

	// else if ( !circling[closest[0]] && 
	// 	(
	// 	(( !q[0] &&  q[1] && !q[2] && !q[3] && !q[4] &&  q[6] )) ||   // not link 3
	// 	(( !q[0] && !q[1] && !q[2] &&  q[3] && !q[4] &&  q[6] ))    // not link 3
	// 	)
	// 	)
	// {
	// 	if (dim == 1)
	// 		v += 0.1;
	// 			alreadydone[ID] = false;

	// }

	// else if ( !circling[closest[0]] && 
	// 	(
	// 	(( !q[0] &&  q[1] && !q[2] &&   q[4] && !q[6] && !q[7])) ||   // not link 3
	// 	(( !q[0] && !q[1] && !q[2] &&   q[4] && !q[6] &&  q[7]))    // not link 3
	// 	)
	// 	)
	// {
	// 	if (dim == 0)
	// 		v += 0.1;
	// 			alreadydone[ID] = false;

	// }

	else if ( !circling[closest[0]])
	{
		circling[ID] = false; // lattice || static

		attractionmotion ( dim, v_r + v_adj, v_b, v);
		latticemotion    ( dim, v_adj      , v_b, bdes[minindex], v);
		alreadydone[ID] = false;
		
	}
	else	
	{
		circling[ID] = false; // lattice || static
		attractionmotion (dim,v_r,v_b,v);
		alreadydone[ID] = false;
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