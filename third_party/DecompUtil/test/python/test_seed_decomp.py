from decomp_util_2d import *
from geometric_utils import *
import numpy as np
import matplotlib.pyplot as plt

obs = np.array([[-0.2, 1.5],
                [1, 0],
                [0.8, -1],
                [-0.5, -0.5]])

s_pos = np.array([0, 0])
bbox = np.array([4, 4])

decomp = SeedDecomp2D(s_pos)
decomp.set_obs(obs)
decomp.set_local_bbox(bbox)
decomp.dilate(1.)

# print(decomp.get_ellipsoid())
poly = decomp.get_polyhedron()
E = decomp.get_ellipsoid()
vertices = np.array(cal_vertices(poly))
vertices = np.vstack([vertices, vertices[0]])

plt.axis('equal')
plt.scatter(obs[:,0], obs[:, 1], c='r')
plt.plot(vertices[:, 0], vertices[:, 1])
plt.show()