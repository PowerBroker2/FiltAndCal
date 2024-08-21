import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt

from numpy.random import MT19937
from numpy.random import RandomState, SeedSequence


np.set_printoptions(threshold=np.inf, linewidth=1000)


DF = 3  # Degrees of freedom (demensionality)
N  = 20 # Number of samples

MEAN_MEAN  = 2
MEAN_SCALE = 3

SEED = 123456789


# https://stackoverflow.com/a/14958796/9860973
import numpy as np
import numpy.linalg as linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

rs = RandomState(MT19937(SeedSequence(SEED)))
mean = ((rs.rand(DF) - 0.5) * MEAN_SCALE) + MEAN_MEAN
cov = rs.multivariate_normal(np.zeros((DF,)), np.eye(DF), DF)
cov = cov.T @ cov
data = rs.multivariate_normal(mean, cov, N).T
print(data)

cov = np.cov(data)
mean = np.mean(data, axis=1)
print(cov)
print(mean)

# your ellispsoid and center in matrix form
A = la.inv(cov)
center = mean

# find the rotation matrix and radii of the axes
U, s, rotation = linalg.svd(A)
radii = 1.0/np.sqrt(s)

# now carry on with EOL's answer
u = np.linspace(0.0, 2.0 * np.pi, 100)
v = np.linspace(0.0, np.pi, 100)
x = radii[0] * np.outer(np.cos(u), np.sin(v))
y = radii[1] * np.outer(np.sin(u), np.sin(v))
z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
for i in range(len(x)):
    for j in range(len(x)):
        [x[i,j],y[i,j],z[i,j]] = (np.dot([x[i,j],y[i,j],z[i,j]], rotation))*2 + center

# plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color='b', alpha=0.2)
ax.scatter(data[0, :], data[1, :], data[2, :])
plt.show()
plt.close(fig)
del fig