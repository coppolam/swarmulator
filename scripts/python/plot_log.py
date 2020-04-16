import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import sys

def get_latest_file():
	list_of_files = glob.glob('../../logs/*.txt') # * means all if need specific format then *.csv
	latest_file = max(list_of_files, key=os.path.getctime)
	return latest_file

if np.size(sys.argv) == 1:
	file = get_latest_file()
	print("No log indicted, using latest log.")
else:
	file = sys.argv[1]

print("Using log: " + file)
data = np.loadtxt(file)

r_time = 0
r_ids = 1
r_x = 2
r_y = 3

n_agents = int(data[:,1].max())
print("Total number of robots: " + str(n_agents))

from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = fig.gca(projection='3d')
for x in range(1,n_agents):
	d = data[np.where(data[:,r_ids] == x)]
	ax.plot(d[:,r_time],d[:,r_x],d[:,r_y])
ax.set_xlabel("Time[s]")
ax.set_ylabel("x")
ax.set_zlabel("y")
plt.show()