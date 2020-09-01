"""
evolving a neural network for source seeking
"""

import sys, pickle, argparse, random
sys.path.insert(1,'../flat_game')
sys.path.insert(1,'../')
sys.path.insert(1,'../settings')
import numpy as np
from pygmo import *
from tools import fileHandler as fh
from classes import evolution, swarmulator
from classes.randomize_environment import get_spawn_pos

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

print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=False)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
# Swarmulator settings
sim.runtime_setting("time_limit", str("50")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("300")) # Real time factor
sim.runtime_setting("environment", "image_testing") # Environment, leave empty for boundless
sim.runtime_setting("fitness", "source_distance") # Fitness function to use (in sw/simulation/fitness_functions.h)


# Specify network topology
shape_file = "../../conf/policies/gas_shape.txt"
policy_file = "conf/policies/gas_params.txt"
sim.runtime_setting("policy", policy_file) 
environments = ['rand_env_1','rand_env_2','rand_env_3','rand_env_4','rand_env_5']

policy_shape = [6,20,20,3]
num_params = 0
bias_add = True
num_agents = 10
sim.set_n_agents(num_agents)
num_params+= np.sum([policy_shape[i]*policy_shape[i+1] for i in range(len(policy_shape)-1)])
if(bias_add):
    num_params+= np.sum(policy_shape[1:])
fh.save_to_txt(np.array(policy_shape),shape_file)


class prob_bart:
    
    def __init__(self):
        self.dim = num_params
    
    def fitness(self,x):
        fh.save_to_txt(x, sim.path+policy_file)
        f = sim.batch_run_envs(environments) # Run with 10-20 robots, 5 times (default args.batchsize=5)
        return [f.mean()]

    def get_bounds(self):
        return([-20]*num_params,[20]*num_params)


if __name__ == "__main__":
    
    
    algo = algorithm(sga(gen=1))
    algo.set_verbosity(1)
    prob = problem(prob_bart())
    pop = population(prob,50)


    for i in range(400):
        get_spawn_pos(num_agents,'../../conf/environments/')
        print("Generation %i"%i)
        pop = algo.evolve(pop)
        print(pop.champion_f)
        fh.save_to_txt(pop.champion_x,'best_individual.txt')

    print(pop)



    uda = algo.extract(sga)
    log = uda.get_log()
    #print(log)
    import matplotlib.pyplot as plt 
    plt.plot([entry[0] for entry in log],[entry[2]for entry in log], 'k--') 
    plt.show() 
    #print(model.nn.get_weights())