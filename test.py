from cloudDistance import DistFunc
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

print("Loading and making data structure")
labcorn_largeall = DistFunc('/media/gulf/corn/cropped_plants/norm_filtered/all_plant5.ply', offset=0.05)
print("done")

print("sampling points")
X, dists = labcorn_largeall.random_sample(1000)
print("done")
inds = dists < 0

print("Plotting")
fig = plt.figure()
ax = Axes3D(fig)

#ax.plot3D(X[ind, 0], y_line, z_line, 'gray')

ax.scatter3D(X[inds, 0], X[inds, 1], X[inds, 2], marker='.');

plt.show()
