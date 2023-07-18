from decomp_util_2d import *
from geometric_utils import *
import numpy as np
import matplotlib.pyplot as plt

obs = np.array([[-0.2, 1.5],
                [0, 1.5],
                [0, 1],
                [1, 0],
                [1.8, 0],
                [0.8, -1],
                [-0.5, -0.5],
                [-0.75, -0.5],
                [-1, -0.5],
                [-1, 0.8]])

origin = np.array([-2, -2])
range = np.array([4, 4])

bbox = np.array([2, 2])

path = np.array([[1, 1.0],
                 [0.0, 0.0],
                 [-1, 1.0]])

decomp = EllipsoidDecomp2D(origin, range)
decomp.set_obs(obs)
decomp.set_local_bbox(bbox)
decomp.dilate(path, 0)

# print(decomp.get_ellipsoid())
poly = np.array(decomp.get_polyhedrons())
E = decomp.get_ellipsoids()
vertices = []
for i in poly:
    vertice = np.array(cal_vertices(i))
    vertice = np.vstack([vertice, vertice[0]])
    vertices.append(vertice)
vertices = np.array(vertices)

plt.axis('equal')
plt.scatter(obs[:,0], obs[:, 1], c='red')
for vertice in vertices:
    plt.plot(vertice[:, 0], vertice[:, 1], c='green')
plt.plot(path[:, 0], path[:, 1], c='blue')
plt.show()
