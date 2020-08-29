import pickle, argparse
import numpy as np
import random, sys, matplotlib
import matplotlib.pyplot as plt
from deap import base, creator, tools
from tools import fileHandler as fh

## Run as
# python3 main_standard_evolution.py CONTROLLER AGENT
# Example:
# python3 main_standard_evolution.py aggregation particle

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Evolve a controller using swarmulator')
parser.add_argument('filename', type=str, help="(str) .pkl file to load")
parser.add_argument('output_file',type=str,help="(str) .txt file to write winning policy to")
args = parser.parse_args()

class Pickle_loader:
    def __init__(self):
        self.cp = pickle.load(open(args.filename, "rb"))
        self.stats = self.cp["stats"]
        self.pop = self.cp["population"]
        
        self.gens = [i['g'] for i in self.stats]
        self.mu = [i['mu'] for i in self.stats]
        self.min = [i['min'] for i in self.stats]
        self.max = [i['max'] for i in self.stats]

    def plot(self):
        plt.plot(self.gens,self.mu,self.gens,self.min,self.gens,self.max)
        plt.xlabel("Number of Generations")
        plt.ylabel("Fitness")
        plt.legend(["Average","Min","Max"])
        plt.show()

    def write_best(self):
        self.winner = tools.selBest(self.pop,1)[0]
        fh.save_to_txt(self.winner,args.output_file)

if __name__ == "__main__":
    pickle = Pickle_loader()
    pickle.plot()
    pickle.write_best()
    