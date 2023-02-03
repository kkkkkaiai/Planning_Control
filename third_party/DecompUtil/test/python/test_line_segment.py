from decomp_util import *
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

pos1 = np.array([-1.5, 0.0])
pos2 = np.array([1.5, 0.3])

bbox = np.array([2, 2])

decomp = LineSegment(pos1, pos2)
decomp.set_obs(obs)
decomp.set_local_bbox(bbox)
decomp.dilate(0)

# print(decomp.get_ellipsoid())
poly = decomp.get_polyhedron()
E = decomp.get_ellipsoid()
vertices = np.array(cal_vertices(poly))
vertices = np.vstack([vertices, vertices[0]])
line_segment = np.array(decomp.get_line_segment())

plt.axis('equal')
plt.scatter(obs[:,0], obs[:, 1], c='r')
plt.plot(vertices[:, 0], vertices[:, 1])
plt.plot(line_segment[:, 0], line_segment[:, 1], c='blue')
plt.show()