import random, sys, pickle, argparse
import numpy as np
from tools import fileHandler as fh
from classes import evolution, swarmulator

params_file = "conf/policies/gas_params.txt"
shape_file = "conf/policies/gas_shape.txt"
policy_shape = [5,20,20,3]
num_params = 0
bias_add = True

# computing number of necessary parameters to describe network
num_params+= np.sum([policy_shape[i]*policy_shape[i+1] for i in range(len(policy_shape)-1)])
if(bias_add):
    num_params+= np.sum(policy_shape[1:])

params = np.random.uniform(-20,20,num_params)

fh.save_to_txt(params,params_file)
fh.save_to_txt(np.array(policy_shape),shape_file)