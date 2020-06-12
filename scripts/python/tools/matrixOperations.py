#!/usr/bin/env python3
"""
Collection of tools for "less standard" math operations
@author: Mario Coppola, 2020
"""
import numpy as np

def round_to_multiple(a, mult, floor=True):
	'''Rounds a number to its multiple'''
	if floor:
		a = np.floor(a / mult)
	else:
		a = np.round(a / mult)
	return a * mult

def normalize_rows(mat,axis=1):
	'''Normalzies the rows of a matrix'''
	row_sums = np.sum(mat, axis=axis)
	if not np.isscalar(row_sums):
		mat = np.divide(mat,row_sums[:,np.newaxis], out=np.zeros_like(mat), where=row_sums[:,np.newaxis]!=0)
	else:
		mat = np.divide(mat,row_sums)
	return mat

def pagerank(G, tol=1e-8):
	'''Iterative procedure to solve for the PageRank vector'''
	G = normalize_rows(G)
	n = G.shape[0]
	pr = 1 / n * np.ones((1, n)) # Initialize PageRank vector
	residual = 1 # Initialize residual
	
	while residual >= tol:
		pr_previous = pr
		pr = np.matmul(pr,G) # Pagerank formula
		residual = np.linalg.norm(np.subtract(pr,pr_previous))
		# print(residual)
	
	return normalize_rows(np.asarray(pr))

def pretty_print(mat):
	'''Prints a matrix to the terminal but it looks a little better'''
	for x in mat:
		print(*x, sep=" ")
