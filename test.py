from cloudDistance import DistFunc
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

print("Loading and making data structure")
labcorn_largeall = DistFunc('/media/gulf/corn/labcorn_largesingle/dense/single_filtered_crop.ply', offset=0.1)
print("done")

print("sampling points")
X = np.random.randn(100000,3)
dists = labcorn_largeall.sample_dist(X)
print("done")
inds = dists < 0

print("Plotting")
fig = plt.figure()
ax = Axes3D(fig)

#ax.plot3D(X[ind, 0], y_line, z_line, 'gray')

ax.scatter3D(X[inds, 0], X[inds, 1], X[inds, 2], marker=',');

plt.show()
