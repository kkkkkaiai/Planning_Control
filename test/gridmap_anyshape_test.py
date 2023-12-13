"""
author: Kai CHEN
mail: chenkai0130@outlook.com
license: MIT
"""
import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from matplotlib.collections import LineCollection
from matplotlib.path import Path
from matplotlib.patches import Polygon, PathPatch, FancyArrow
from envs.gridmap import OccupancyGridMap
from planner.path_generator.astar import *
from planner.path_generator.path_manager import PathManager
from planner.trajectory_generator.spline_interpolate import *
from utils.util import *
from sensors.laser_anyshape import Laser
from decomp_util_2d import *
from geometric_utils import *

if __name__ == '__main__':
    map_file='envs/maps/obstacle_map_occupancy.png'
    resolusion = 0.05
    gridmap = OccupancyGridMap.from_png(map_file, resolusion)
    pM = PathManager(gridmap)
    start_position = np.array([1.0, 1.0])
    end_position = np.array([4.5, 5.2])
    path = pM.find_path(start_position, end_position)
    # path = pM.find_gradient()
    pM.spline_interpolate(path)

    robot_yaw = 0
    robot_radius = 0.3
    laser = Laser(beams=256)
    laser.set_map(gridmap)
    bbox = np.array([1.5, 1.5])/resolusion

    robot_instance = None
    convex_vertice = None
    laser_points = None
    arrow_instance = None

    def update_robot_plot(location, re=False):
        global robot_instance
        if not re:
            robot_instance = plt.Circle((location[0]/resolusion, location[1]/resolusion), robot_radius/resolusion, color='tab:gray', fill=False)
        else:
            robot_instance.set_center((location[0]/resolusion, location[1]/resolusion))
        return robot_instance
    
    def update_convex_vertice(vertice, re=False):
        global convex_vertice
        if not re:
            convex_vertice = Polygon(vertice, color='orange', fill=False)
        else:
            convex_vertice.set_xy(vertice)
        return convex_vertice
    
    def update_laser_points(points, re=False):
        global laser_points

        if not re:
            laser_points = PathPatch(Path(points), color='red', fill=False, )
        else:
            laser_points.set_path(Path(points))
        return laser_points
    
    def update_arrow_plot(location, re=False):
        global arrow_instance
        if not re:
            dx = np.cos(robot_yaw) * robot_radius/resolusion*0.5
            dy = np.sin(robot_yaw) * robot_radius/resolusion*0.5
            arrow_instance = FancyArrow(location[0]/resolusion, location[1]/resolusion, dx, dy, width=0.01, head_width=2, color='tab:green')
        else:
            dx = np.cos(robot_yaw) * robot_radius/resolusion*0.5
            dy = np.sin(robot_yaw) * robot_radius/resolusion*0.5
            arrow_instance.set_data(x=location[0]/resolusion, y=location[1]/resolusion, dx=dx, dy=dy)
        return arrow_instance

    pM.plot_map()
    pM.plot_interpolated_path()
    pM.plot_path()
    ax = plt.gca()
    ax.add_patch(update_robot_plot(start_position*resolusion))
    ax.add_patch(update_convex_vertice([[0, 0]]))
    # ax.add_patch(update_laser_points([[0, 0]])
    ax.add_patch(update_arrow_plot(start_position*resolusion))

    obs_points = None
    obs_path = None
    ver_sets = None

    for index in range(len(path)):
        robot_c = path[index]*resolusion
        de_obs = laser.state2obs(robot_c, robot_yaw)
    
        decomp = SeedDecomp2D(np.array(robot_c)/resolusion)
        decomp.set_obs(de_obs['point'])
        decomp.set_local_bbox(bbox)
        decomp.dilate(1.)
        poly = decomp.get_polyhedron()
        E = decomp.get_ellipsoid()
        vertices = np.array(cal_vertices(poly))
        vertices = np.vstack([vertices, vertices[0]])

        # plot laser point 
        # if obs_points is None:
        #     obs_points = plt.scatter(de_obs['point'][:, 0], de_obs['point'][:, 1], c='tab:blue', s=0.5)
        #     print(type(obs_points.get_paths()))
        # else:
        #     obs_points = plt.scatter(de_obs['point'][:, 0], de_obs['point'][:, 1], c='tab:blue', s=0.5)
            # print(point_data)
        # obs_points.set_paths(point_data)


        update_robot_plot(robot_c, True)
        update_convex_vertice(vertices, True)
<<<<<<< HEAD
        update_laser_points(de_obs['point'], True)
=======
        # update_laser_points(de_obs['point'], True)
        update_arrow_plot(robot_c, True)
>>>>>>> 13403551a6745c8884347d89cfa0fe1135e09e2d

        plt.legend()
        plt.tight_layout()
        plt.axis('off')
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
        plt.pause(0.0001) # pause a bit so that plots are updated
        
        
