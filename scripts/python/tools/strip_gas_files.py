## This script takes a folder of gas reading files and strips it down to a specific altitude, converts 3d space to 2d space

import random, sys, pickle, argparse, cv2, imutils, os, glob
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from itertools import groupby, cycle 


#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Convert 3D gas data to 2D to save space')
parser.add_argument('-env_name', type=str, help="(str) Folder name in /conf/environments containing the environment that needs to be stripped ",default='image_testing')
parser.add_argument('-alt',type=int, help="(int) integer count in z-direction, if alt=0 gas is recorded on ground level. Maximum alt is dependent on mesh",default=0)
args = parser.parse_args()

def clean_folder(folder):
    gas_dir = "conf/environments/" + args.env_name + "/gas_simulations/"
    alt = args.alt
    for file in glob.glob(gas_dir+"*.txt"):
        f = open(file,"r")
        f = f.readlines()
        cleaned_lines = []
        cleaned_lines = cleaned_lines + f[:7]
        for line in f[7:]:
            if (int(line.rstrip().split(" ")[2])==alt):
                cleaned_lines.append(line)  
        
        f = open(file,"w")
        f.writelines(cleaned_lines)
        


        



if __name__ == "__main__":
    clean_folder(args.env_name)