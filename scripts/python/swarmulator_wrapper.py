#!/usr/bin/env python3
"""
Example python interface with swarmulator
@author: Mario Coppola, 2020
"""

from simulator import swarmulator
from tools import room_generator

path = "../swarmulator" # Path to swarmulator main folder
robots = 20 # Number of robots
s = swarmulator.swarmulator(path) # Initialize
s.make(clean=False,animation=True) # (Re)-build (if swarmulator already built, you can skip this)
f = s.run(robots) # Run it, and receive the fitness as a FIFO pipe message

print("Fitness received from swarmulator pipe: " + str(f))
print("\t...done!")
