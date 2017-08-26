#ifndef SIMULATION_H
#define SIMULATION_H
#include "parameters.h"
#include "randomgenerator.h"
#include <numeric>
#include <string>
#include <functional>
#include "omniscient_observer.h"
#include <cctype>
#include <algorithm>


void run_simulation()
{
	// static OmniscientObserver *obs = new OmniscientObserver();

	mtx.lock();
	for (int i = 0; i < nagents; i++)
	{
		s[i].update_position();
	}

	// obs->adjacency_matrix(); // Calculate the adjacency matrix
	mtx.unlock();

	int t_wait = (int) 1000000.0*(1.0/(simulation_updatefreq*simulation_realtimefactor));
	this_thread::sleep_for(chrono::microseconds( t_wait ));
	simulation_time += t_wait;
	simtime_seconds = simulation_realtimefactor*simulation_time/1000000.0;
}

/* Calculate the mean of a vector element */
float vector_mean(const vector<float> &v)
{
	float sum = std::accumulate(v.begin(), v.end(), 0.0);
	return sum / v.size();
}

/* Calculate the standard deviation of a vector element */
float vector_std(const vector<float> &v)
{
	std::vector<double> diff(v.size());
	std::transform(
		v.begin(), v.end(), diff.begin(),
		std::bind2nd(std::minus<double>(), vector_mean(v))
		);
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	return sqrt(sq_sum / v.size());
}

void start_simulation(int argc, char* argv[]);

void simulation_start()
{
	simulation_time = 0.0; // Initialize simulation time to zero.

	while(true)
		{ run_simulation(); };
}

/* Generate a random vector with zero mean */
vector<float> generate_random_vector(const int &length, mt19937 e2,normal_distribution<float> rn)
{
	// Generate the random vector
	std::vector<float> v(length);
	for (int i = 0; i < length; i++)
	{
    	    v[i] = getrand_float(-0.5,0.5);
    }

    // Adjust to zero mean
    vector<float> temp = v;
    for (int i = 0; i < length; i++)
    {
    	v[i] = ((v[i]-vector_mean(temp)));
    }
    
    return v;
}

void start_simulation(int argc, char* argv[]){

	// Extract number of agents from the argument
    if(argc<=1)
    {
        printf("Please specify the amount of agents.\n\n");
        exit(1);
    }
    else
    {
    	nagents = stoi(argv[1]);
		s.reserve(nagents); // Reserve spots for the expected amount of agents
    }


	if (argc<=2)
	{
	    printf("No nearest-neighbor rule specified. Assuming full connectivity. \n\n");
    	knearest = nagents-1;
    }
    else 
    {
    	knearest = stoi(argv[2]);
    	
    	if (knearest > (nagents-1))
    	{
		    printf("You can't have more nearest-neighbors that the number of observable agents. Fix. \n\n");
        	exit(1);
    	}
	}

    // Generate random initial positions
    // TODO: Make this code a bit more elegant.
	void randomgen_init();
	srand ( time ( NULL ) );
	normal_distribution<float> rn(0.0, 1.0);
	random_device rd1; // Initialize x
	static mt19937 e2(rd1());
	random_device rd2; // Initialize y
	static mt19937 e1(rd2());
	std::vector<float> x0 = generate_random_vector(nagents,e2,rn);
 	std::vector<float> y0 = generate_random_vector(nagents,e1,rn);

	// Set here the model. This main should just spawn n agents at random positions/states.
	for (int i = 0; i < nagents; i++)
	{
		vector<float> states = { x0[i], y0[i], 0.0, 0.0};  // Initial positions/states
		s.push_back(Particle(i,states,1.0/simulation_updatefreq));
	}

	simulation_start(); // Begin the simulation!
}
#endif /*SIMULATION_H*/