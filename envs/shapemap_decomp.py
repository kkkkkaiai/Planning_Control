'''
Reference: https://github.com/taewankim1/sequential_convex_programming
Author: taewankim1, Kai CHEN
License: MIT
'''

import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

import numpy as np
import math
import time
import os
import matplotlib.pyplot as plt
from models.python.geometry_utils import PolytopeRegion, RectangleRegion
from envs.gridmap import OccupancyGridMap

from decomp_util_2d import *
from geometric_utils import *

def gen_obs_map(file_path="envs/maps/obstacle_map_occupancy.png", resolusion=0.05, num_obs=None, radius_range=[0.1, 0.25], x_range=[-1.0, 1.0], y_range=[0.7, 5.3], obs_dist=0.5, load_mode=True):
    if os.path.exists(file_path):
        return
    
    edges = [-2.5, 2.5, -1, 7]
    edge_width = 0.1
    left_e, right_e, down_e, up_e = edges

    plt.figure(figsize=(5,8), facecolor='k', dpi=26)
    ax = plt.gca()

    circle_list = []
    max_iter = 50

    def euclidean_distance(x1, y1, x2, y2):
        '''
        calculate the euclidean distance between two points
        '''
        return math.hypot((x1 - x2), (y1 - y2))

    if num_obs is None :
        num_obstacle = np.random.randint(4,8)
    else : 
        num_obstacle = num_obs

    for i in range(max_iter) :
        if i == max_iter -1 or len(circle_list) == num_obstacle:
            break

        # generate random circle in a certain range
        r = np.random.uniform(radius_range[0], radius_range[1])
        x = np.random.uniform(x_range[0], x_range[1])
        y = np.random.uniform(y_range[0], y_range[1])
        if not any((x2, y2, r2) for x2, y2, r2 in circle_list if euclidean_distance(x, y, x2, y2) < r + r2 + obs_dist):
            circle_list.append((x, y, r))  
            circle = plt.Circle((x, y), r, color='white', alpha=1.0, fill=True)
            ax.add_patch(circle)

    polytope = PolytopeRegion.convex_hull(np.array([[0, 3], [1,2.0], [1.5, 2.2], [2.0, 3.2], [1.2, 3.0]]))
    poly_patch = polytope.get_plot_patch(ec="white", fc="white")
    ax.add_patch(poly_patch)

    # add edge patch
    left_rec = RectangleRegion(left_e, left_e+edge_width, up_e, down_e)
    right_rec = RectangleRegion(right_e-edge_width, right_e, up_e, down_e)
    down_rec = RectangleRegion(left_e, right_e, down_e+edge_width, down_e)
    up_rec = RectangleRegion(left_e, right_e, up_e, up_e-edge_width)
    left_patch = left_rec.get_plot_patch(ec="white", fc="white")
    right_patch = right_rec.get_plot_patch(ec="white", fc="white")
    down_patch = down_rec.get_plot_patch(ec="white", fc="white")
    up_patch = up_rec.get_plot_patch(ec="white", fc="white")
    
    ax.add_patch(left_patch)
    ax.add_patch(right_patch)
    ax.add_patch(down_patch)
    ax.add_patch(up_patch)

    plt.axis(edges)
    plt.axis('off')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig('envs/maps/obstacle_map_occupancy.png', bbox_inches='tight', pad_inches=0)
    # plt.show()

def test():
    gen_obs_map()

if __name__ == '__main__':
    test()