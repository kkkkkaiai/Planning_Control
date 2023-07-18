"""
@author: Kai Chen
"""
from geometry_utils import SuperEllipse, RectangleRegion, PolytopeRegion, get_dist_point_to_region, get_dist_region_to_region, 
import numpy as np
import matplotlib.pyplot as plt
import time


def test_ellipse():
    s = SuperEllipse([2.5, 1.8], 0.5, None, 45, 20)
    s.plot()
    plt.show()

def test_dist():
    # # PolytopeRegion test
    obs_1 = RectangleRegion(0.0, 1.0, 0.90, 1.0)
    A, b = obs_1.get_convex_rep()
    # formulate to polytope region
    obs_2 = PolytopeRegion(A, b)
    obs_3 = PolytopeRegion.convex_hull(obs_2.points)
    # print('obs3 mat A', obs_3.mat_A.T)
    # print('obs3 vec b', obs_3.vec_b.T)
    # print(all([all(obs_3.points[:, i] == obs_2.points[:, i]) for i in range(2)]))
    obs_4 = PolytopeRegion.convex_hull(np.array([[3, 0], [4, 1], [5, 3], [3.5, 5], [0, 3]]))
    A, b = obs_4.get_convex_rep()

    # dual variable test in distance functions
    # point_to_region
    points = np.array([[0.0, 2.0, 3.0], [0.0, 2.0, 3.0 ]])
    dists = []
    start_time = time.time()
    for i in range(points.shape[1]):
        point = np.array([[points[0, i]], [points[1, i]]])
        dists.append(get_dist_point_to_region(point, A, b))
    end_time = time.time()
    print('time ', end_time - start_time)
    print('dist ', dists)

    fig, ax = plt.subplots()
    patch_2 = obs_2.get_plot_patch()
    patch_4 = obs_4.get_plot_patch()
    ax.add_patch(patch_2)
    ax.add_patch(patch_4)

    dist_1 = get_dist_point_to_region(point, A, b)
    
    # print('dual 1 ', dual_1)
    # before normalization of dual in distance function
    # print(np.sqrt(-0.25*dual_1@A@A.T@dual_1 + dual_1@(A@point - b)))
    # after normalization of dual in distance function
    # print(dual_1 @ (A @ point - b))
    # print(np.linalg.norm(dual_1 @ A), "\n")

    plt.xlim([-5, 10])
    plt.ylim([-5, 10])
    plt.show()

test_dist()