"""
evolving a neural network for source seeking
"""

import sys, pickle, argparse, random
sys.path.insert(1,'../flat_game')
sys.path.insert(1,'../')
sys.path.insert(1,'../settings')
import numpy as np
from flat_game import carmunk
from settings import *
from pygmo import *
import keras
from keras.models import Sequential
from keras.layers import Dense
from tools import fileHandler as fh
from classes import evolution, swarmulator



params = [20,20,NUM_ACTIONS]
feed_forward = True


class prob_bart:
    
    def __init__(self):
        self.dim = model.num_elements
    
    def fitness(self,x):
        
        model.unfold_vector(x)
        model.set_nn_params(model.unfolded_params)
        
        #return [(np.sum(model.evaluate(np.array([1,2,3,4])))).item()]
        a = [game_state.evaluate_model(model.nn).item()]
        #print(a)
        #a = [(np.sum(model.evaluate(np.array([1,2,3,4])))).item()]
        return a

    def get_bounds(self):
        return([-20]*model.num_elements,[20]*model.num_elements)


if __name__ == "__main__":
    
    
    algo = algorithm(sga(gen=1))
    algo.set_verbosity(1)
    prob = problem(prob_bart())
    pop = population(prob,50)

    lower_alpha = 0.1
    higher_alpha = 0.9

    lower_freq = 5
    higher_freq = 10

    for i in range(400):
        print("Generation %i"%i)
        # alpha = random.random()*(higher_alpha-lower_alpha)+lower_alpha
        # freq = int(random.random()*(higher_freq-lower_freq)+lower_freq)

        # game_state.gas_sensor.alpha = alpha
        # game_state.gas_sensor.set_sensor_freq(freq)
        pop = algo.evolve(pop)
        game_state.randomize_agents()
        print(pop.champion_f)
        
        if i%5 ==0:
            model.unfold_vector(pop.champion_x)
            model.set_nn_params(model.unfolded_params)   
            model.nn.save('../saved-models/model.h5')            

    print(pop)


    model.unfold_vector(pop.champion_x)
    model.set_nn_params(model.unfolded_params)   
    model.nn.save('../saved-models/model.h5')
    
    uda = algo.extract(sga)
    log = uda.get_log()
    #print(log)
    import matplotlib.pyplot as plt 
    plt.plot([entry[0] for entry in log],[entry[2]for entry in log], 'k--') 
    plt.show() 
    #print(model.nn.get_weights())