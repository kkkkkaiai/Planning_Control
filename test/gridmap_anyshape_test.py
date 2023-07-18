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
from matplotlib.patches import Polygon, PathPatch
from envs.gridmap import OccupancyGridMap
from planner.path_generator.astar import *
from planner.trajectory_generator.spline_interpolate import *
from utils.util import *
from sensors.laser_anyshape import Laser
from decomp_util_2d import *
from geometric_utils import *

class PathManager:
    def __init__(self, gridmap, start_node=(6.0, 3.0), end_node=(16.0, 16.0)) -> None:
        self._map = gridmap
        self._resolution = self._map.cell_size
        self._start_node = start_node
        self._end_node = end_node
        self._path = None
        self._path_px = None

    def find_path(self, start_node, end_node):
        # run A*
        self._start_node = start_node
        self._end_node = end_node
        self._path, self._path_px = a_star(self._start_node, self._end_node, self._map, movement='8N')
        self._path = np.asarray(self._path) / self._resolution
        return self._path
    
    def find_gradient(self):
        '''
        find the index of which the gradient is larger than a threshold
        '''
        path_index = []
        for i in range(1, len(self._path)-1):
            if abs(np.linalg.norm(self._path[i]-self._path[i-1]) - \
                   np.linalg.norm(self._path[i+1]-self._path[i])) > 0.2:
                path_index.append(i-1)
        path_index.append(len(self._path)-1)
        return self._path[path_index]
    
    def is_path_found(self):
        return self._path is not None
    
    def spline_interpolate(self, path=None, ds=0.1):
        if path is None:
            path = self._path
        cx, cy, cyaw, ck, s = calc_spline_course(path[:, 0]*self._resolution, path[:, 1]*self._resolution, ds=ds)
        self._sp = calc_speed_profile(cx, cy, cyaw, 10)
        self._ref_path = PATH(cx, cy, cyaw, ck)
        print('Path length: ', len(path), 'Interpolated path length: ', self._ref_path.length)
        return self._ref_path, self._sp

    def plot_map(self):
        self._map.plot()

    def plot_path(self):
        if self.is_path_found():
            plot_path(self._path, linestyle='--', label='origin path')
        else:
            # plot start and goal points over the map (in pixels)
            start_node_px = self._map.get_index_from_coordinates(self._start_node[0], self._start_node[1])
            goal_node_px = self._map.get_index_from_coordinates(self._end_node[0], self._end_node[1])

            plt.plot(start_node_px[0], start_node_px[1], 'ro')
            plt.plot(goal_node_px[0], goal_node_px[1], 'go')
            raise ValueError('Path is not found')
        
    def plot_interpolated_path(self):
        try:
            if self._ref_path is not None:

                plot_path(np.array([np.array(self._ref_path.cx)/self._resolution, np.array(self._ref_path.cy)/self._resolution]).T, \
                          color='cyan', label='interpolated path')
        except:
            print('Not do interpolation yet, please call spline_interpolate() first')


    

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

    robot_yaw = np.pi
    robot_radius = 0.3
    laser = Laser(beams=128)
    laser.set_map(gridmap)
    bbox = np.array([1.5, 1.5])/resolusion

    robot_instance = None
    convex_vertice = None
    laser_points = None

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
            laser_points = PathPatch(points, color='red', fill=False, )
        else:
            laser_points.set_path(Path(points))
        return laser_points
    # def update_arrow_plot(location):
    #     dx = np.cos(robot_yaw) * robot_radius/resolusion
    #     dy = np.sin(robot_yaw) * robot_radius/resolusion
    #     arrow = plt.Arrow(location[0]/resolusion, location[1]/resolusion, dx, dy, color='tab:green')
    #     return arrow

    pM.plot_map()
    pM.plot_interpolated_path()
    pM.plot_path()
    ax = plt.gca()
    ax.add_patch(update_robot_plot(start_position*resolusion))
    ax.add_patch(update_convex_vertice([[0, 0]]))
    ax.add_patch(update_laser_points(Path([[0, 0]])))
    # ax.add_patch(update_arrow_plot(start_position*resolusion))

    obs_points = None
    obs_path = None
    ver_sets = None

    for index in range(len(path)):
        robot_c = path[index]*resolusion
        de_obs = laser.state2obs(robot_c, robot_yaw)
    
        decomp = SeedDecomp2D(np.array(robot_c)/resolusion)
        decomp.set_obs(de_obs['point'])
        print(de_obs['point'])
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
        # update_laser_points(de_obs['point'], True)

        plt.legend()
        plt.tight_layout()
        plt.axis('off')
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
        plt.pause(0.0001) # pause a bit so that plots are updated
        
        
