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

from matplotlib.path import Path
from matplotlib.patches import PathPatch, Circle
from envs.gridmap import OccupancyGridMap
from planner.path_generator.astar import *
from planner.trajectory_generator.spline_interpolate import *
from utils.util import *
from sensors.laser_anyshape import Laser
from decomp_util_2d import *
from geometric_utils import *

from copy import deepcopy

from laser_attractive_point_whole import *
from path_manager import PathManager


if __name__ == '__main__':
    map_file='envs/maps/obstacle_map_occupancy.png'
    resolution = 0.05
    gridmap = OccupancyGridMap.from_png(map_file, resolution)
    pM = PathManager(gridmap)
    start_position = np.array([1.0, 2.0])  # wrong tangent point
    # start_position = np.array([1.0, 2.0])  # wrong tangent point
    end_position = np.array([4.5, 5.2])
    path = pM.find_path(start_position, end_position)
    # path = pM.find_gradient()
    pM.spline_interpolate(path)
    
    robot_yaw = np.pi
    robot_radius = 0.3
    laser = Laser(beams=256)
    laser.set_map(gridmap)
    bbox = np.array([1.5, 1.5])/resolution

    robot_instance = None
    laser_points = None
    travel_position_list = []

    def update_robot_plot(location, re=False):
        global robot_instance
        if not re:
            robot_instance = plt.Circle((location[0]/resolution, location[1]/resolution), robot_radius/resolution, color='tab:gray', fill=False)
        else:
            robot_instance.set_center((location[0]/resolution, location[1]/resolution))
        return robot_instance
    
    def update_laser_points(points, re=False):
        global laser_points
        if not re:
            laser_points = PathPatch(points, color='red', fill=False, )
        else:
            laser_points.set_path(Path(points))
        return laser_points

    ax = plt.gca()
    
    obs_points = None
    obs_path = None
    ver_sets = None
    plt.legend()
    for index in range(len(path)):
        robot_c = path[index]*resolution
        de_obs = laser.state2obs(robot_c, robot_yaw, False)

        travel_position = iplanning(de_obs, robot_c, end_position)

        ax.add_patch(update_robot_plot(start_position*resolution))
        ax.add_patch(update_laser_points(Path([[0, 0]])))

        update_robot_plot(robot_c, True)
        update_laser_points(de_obs['point'], True)

        pM.plot_map()
        pM.plot_interpolated_path()
        pM.plot_path()
        
        plt.tight_layout()
        # save figure with dpi 300
        plt.savefig('test/travel_position/result/{}.png'.format(index), dpi=600)
        plt.cla()

        
        
