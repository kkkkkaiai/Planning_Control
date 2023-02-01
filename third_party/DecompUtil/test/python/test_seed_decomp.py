from decomp_util import *
import numpy as np

obs = list([[-0.2, 1.5],
                [1, 0],
                [0.8, -1],
                [-0.5, -0.5]])

s_pos = np.array([0, 0])

decomp = SeedDecomp(s_pos)
decomp.set_obs(obs)

bbox = np.array([2, 2])
decomp.set_local_bbox(bbox)

decomp.dilate(1.)

# print(decomp.get_ellipsoid())
poly = decomp.get_polyhedron().hyperplanes()

for i in poly:
    print(i.p_)