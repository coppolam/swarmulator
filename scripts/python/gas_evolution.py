import random, sys, pickle, argparse
import numpy as np
from tools import fileHandler as fh
from classes import evolution, swarmulator

## Run as
# python3 main_standard_evolution.py CONTROLLER AGENT
# Example:
# python3 main_standard_evolution.py aggregation particle

num_agents = 10

#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Evolve a controller using swarmulator')
parser.add_argument('-controller', type=str, help="(str) Controller to use", default="gas_seeking")
parser.add_argument('-agent', type=str, help="(str) Swramulator agent to use", default="gas_agent")
parser.add_argument('-gen', type=int, help="(int) Max number generations, default = 100", default=400)
parser.add_argument('-batchsize', type=int, help="(int) Batch size. How many parallel tests to try, default = 5", default=5)
parser.add_argument('-resume', type=str, help="(str) Resume after quitting from the indicated saved file, default = None", default=None)
parser.add_argument('-plot', type=str, help="(str) If set, it will plot the evolution from a saved run, default = None", default=None)
parser.add_argument('-id', type=int, help="(int) Evolutionary run ID, default = 1", default=1)
args = parser.parse_args()

##########################
#  Load Swarmulator API  #
##########################
print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=False)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
# Swarmulator settings
sim.runtime_setting("time_limit", str("50")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("300")) # Real time factor
sim.runtime_setting("environment", "image_testing") # Environment, leave empty for boundless
sim.runtime_setting("fitness", "source_distance") # Fitness function to use (in sw/simulation/fitness_functions.h)
filename = "evo_run_%s_%s_%i" % (args.controller, args.agent, args.id)
print("This run will save at every new generation in the file %s.pkl" % filename)
print("If you want to resume, please load it using the -resume input option.")

######################
#  Fitness function  #
######################
def fitness(individual):
	### Set the policy file that swarmulator reads
	policy_file = "conf/policies/gas_params.txt"
	fh.save_to_txt(individual, sim.path+policy_file)
	sim.runtime_setting("policy", policy_file) # Use random policy

	### Run swarmulator in batches
	# Indicate the minimum number and the maximum number of agents to run with.
	# In this example, a run will feature anything between 10 and 20 robots.
	f = sim.batch_run(num_agents,args.batchsize) # Run with 10-20 robots, 5 times (default args.batchsize=5)
	return f.mean(), # Fitness = average (note trailing comma to cast to tuple!)

########################
#  Load evolution API  #
########################
e = evolution.evolution()

# Specify network topology
shape_file = "../../conf/policies/gas_shape.txt"
policy_shape = [6,20,20,3]
num_params = 0
bias_add = True
num_params+= np.sum([policy_shape[i]*policy_shape[i+1] for i in range(len(policy_shape)-1)])
if(bias_add):
    num_params+= np.sum(policy_shape[1:])
fh.save_to_txt(np.array(policy_shape),shape_file)

# Specify the genome length and the population size
e.setup(fitness, GENOME_LENGTH=num_params, POPULATION_SIZE=50, NUM_AGENTS=num_agents)

# Do not evolve, but only plot an evolution file as specified in args.plot
if args.plot is not None:
    e.load(args.plot)
    e.plot_evolution()

# Resume evolution from file args.resume
elif args.resume is not None:
	e.load(args.resume)
	p = e.evolve(verbose=True, generations=args.gen, checkpoint=filename, population=e.pop)

# Just run normally from the beginning
else:
    p = e.evolve(verbose=True, generations=args.gen, checkpoint=filename)

# Save
e.save(filename)
