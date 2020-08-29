import pickle, argparse
import numpy as np


## Run as
# python3 main_standard_evolution.py CONTROLLER AGENT
# Example:
# python3 main_standard_evolution.py aggregation particle

#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Evolve a controller using swarmulator')
parser.add_argument('filename', type=str, help="(str) .pkl file to load")
args = parser.parse_args()

cp = pickle.load(open(args.filename, "rb"))