#!/usr/bin/env python3
"""
Simulate the aggregation and optimize the behavior
@author: Mario Coppola, 2020
"""
import random, sys, pickle, matplotlib
import numpy as np
import matplotlib.pyplot as plt
from deap import base, creator, tools
matplotlib.rc('text', usetex=True)

class evolution:
	'''Wrapper around the DEAP package to run an evolutionary process with just a few commands'''

	def __init__(self):
		'''Itilize the DEAP wrapper'''
		creator.create("FitnessMin", base.Fitness, weights=(1.0,))
		creator.create("Individual", list, fitness=creator.FitnessMin)

	def setup(self, fitness_function_handle, constraint=None, GENOME_LENGTH = 20, POPULATION_SIZE = 100, P_CROSSOVER = 0.5, P_MUTATION = 0.2):
		'''Set up the parameters'''
		# Set the main variables
		self.GENOME_LENGTH = GENOME_LENGTH
		self.POPULATION_SIZE = POPULATION_SIZE
		self.P_CROSSOVER = P_CROSSOVER
		self.P_MUTATION = P_MUTATION

		# Set the lower level parameters
		self.toolbox = base.Toolbox()
		self.toolbox.register("attr_float", random.random)
		self.toolbox.register("individual", tools.initRepeat, creator.Individual, self.toolbox.attr_float, self.GENOME_LENGTH)
		self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)
		self.toolbox.register("evaluate", fitness_function_handle)
		self.toolbox.register("mate", tools.cxTwoPoint) # Mating method
		self.toolbox.register("mutate", tools.mutFlipBit, indpb=0.05) # Mutation method
		self.toolbox.register("select", tools.selTournament, tournsize=3) # Selection method
		if constraint is not None: self.toolbox.decorate("evaluate", tools.DeltaPenalty(constraint, 7))
		self.stats = [] # Initialize stats vector
	
	def store_stats(self, population, iteration=0):
		'''Store the current stats and return a dict'''
		fitnesses = [ individual.fitness.values[0] for individual in population ]
		return {
			'g': iteration,
			'mu': np.mean(fitnesses),
			'std': np.std(fitnesses),
			'max': np.max(fitnesses),
			'min': np.min(fitnesses)
		}

	def disp_stats(self,iteration=0):
		'''Print the current stats'''
		print(">> gen = %i, mu = %.2f, std = %.2f, max = %.2f, min = %.2f" % 
		(self.stats[iteration]['g'],
		self.stats[iteration]['mu'],
		self.stats[iteration]['std'],
		self.stats[iteration]['max'],
		self.stats[iteration]['min']))

	def plot_evolution(self,figurename=None):
		'''Plot the evolution outcome'''
		plt.style.use('seaborn-whitegrid')
		plt.plot(range(1, len(self.stats)+1), [ s['mu'] for s in self.stats ])
		plt.title('Average fitness per iteration')
		plt.xlabel('Iterations')
		plt.ylabel('Fitness')
		plt.fill_between(range(1, len(self.stats)+1),
					[ s['mu']-s['std'] for s in self.stats ],
					[ s['mu']+s['std'] for s in self.stats ],
					color='gray', alpha=0.2)
		plt.xlim(0,len(self.stats))
		plt.savefig(figurename) if figurename is not None else plt.show()

	def evolve(self, generations=100, verbose=False, population=None, checkpoint=None):
		'''Run the evolution. Use checkpoint="filename.pkl" to save the status to a file after each generation, just in case.'''
		random.seed() # Use random seed
		pop = population if population is not None else self.toolbox.population(n=self.POPULATION_SIZE)
		if verbose: print('{:=^40}'.format(' Start of evolution '))
			
		# Evaluate the initial population
		fitnesses = list(map(self.toolbox.evaluate, pop))
		for ind, fit in zip(pop, fitnesses):
			ind.fitness.values = fit

		# Evolve!
		g = len(self.stats) # Number of generations
		gmax = len(self.stats) + generations
		while g < gmax:
			# Offspring
			offspring = self.toolbox.select(pop, len(pop))
			offspring = list(map(self.toolbox.clone, offspring))
			for child1, child2 in zip(offspring[::2], offspring[1::2]):
				if random.random() < self.P_CROSSOVER: self.toolbox.mate(child1, child2)
				del child1.fitness.values
				del child2.fitness.values

			# Mutate
			for mutant in offspring:
				if random.random() < self.P_MUTATION: self.toolbox.mutate(mutant)
				del mutant.fitness.values
		
			# Evaluate the individuals with an invalid fitness
			invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
			fitnesses = map(self.toolbox.evaluate, invalid_ind)
			for ind, fit in zip(invalid_ind, fitnesses):
				ind.fitness.values = fit
			
			pop[:] = offspring # Replace population
			self.stats.append(self.store_stats(pop, g)) # Store stats
			if verbose: self.disp_stats(g)
			
			if checkpoint is not None: self.save(checkpoint, pop=pop, gen=g, stats=self.stats)

			g += 1

		# Store oucomes
		self.pop = pop
		self.best_ind = self.get_best()
		self.g = g

		# Save to checkpoint file
		if checkpoint is not None: self.save(checkpoint)

		# Display outcome
		if verbose: 
			print('{:=^40}'.format(' End of evolution '))
			print("Best individual is %s, %s" % (self.best_ind, self.best_ind.fitness.values))

		return pop

	def save(self,filename,pop=None,gen=None,stats=None):
		'''Save the current status in a pkl file'''
		p = self.pop if pop is None else pop
		g = self.g if gen is None else gen
		s = self.stats if stats is None else stats
		cp = dict(population=p, generation=g, stats=s)
		with open(filename+".pkl", "wb") as cp_file:
			pickle.dump(cp, cp_file)

	def load(self,filename):
		'''Load the status from a pkl file'''
		with open(filename+".pkl", "rb") as cp_file:
			cp = pickle.load(cp_file)
		self.stats = cp["stats"]
		self.g = cp["generation"]
		self.pop = cp["population"]
		return self.pop

	def get_best(self,pop=None):
		'''Returns the fittest element of a population'''
		p = self.pop if pop is None else pop
		return tools.selBest(p,1)[0]
