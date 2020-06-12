#!/usr/bin/env python3
"""
Collection of functions to handle files, load them, save them, ect
@author: Mario Coppola, 2020
"""
import numpy as np
import os, time
import glob

def load_matrix(file):
	'''Loads a matrix from a file'''
	try:
		matrix = np.loadtxt(open(file, "rb"), delimiter=", \t", skiprows=1)
		return matrix
	except:
		raise ValueError("Matrix " + file + " could not be loaded! Exiting.")

def read_matrix(folder, name, file_format=".csv"):
	mat = load_matrix(folder + name + file_format)
	return mat

def make_folder(folder):
	'''Generates a folder if it doesn not exist'''
	try:
		os.mkdir(folder)
	except:
		None # Directory exists
	folder = folder + "/sim_" + time.strftime("%Y_%m_%d_%T")
	os.mkdir(folder)
	return folder + "/"

def save_data(filename,*args):
	np.savez(filename,*args)
	print("Data saved")

def save_to_txt(mat,name):
	NEWLINE_SIZE_IN_BYTES = -1  # -2 on Windows?
	with open(name, 'wb') as fout:  # Note 'wb' instead of 'w'
		np.savetxt(fout, mat, delimiter=" ", fmt='%.3f')
		fout.seek(NEWLINE_SIZE_IN_BYTES, 2)
		fout.truncate()

def get_latest_file(path):
	list_of_files = glob.glob(path) # * means all if need specific format then *.csv
	latest_file = max(list_of_files, key=os.path.getctime)
	return latest_file
