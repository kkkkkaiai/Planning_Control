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
from sklearn.cluster import DBSCAN

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

def find_travel_position(laser_info):
    '''
    find the travel position
    '''
    laser_points = laser_info['point']
    points_length = laser_info['length']

    # coutour point detection in the point
    travel_position = []
    for i in range(0, len(laser_points)-1):
        # calculate the contour point based on the curvature of the near 5 points
        if (points_length[i] < 0.05) or (points_length[i] > 1.75):
            continue

        cal_range = 3
        low_bound = -cal_range
        upper_bound = cal_range
        diff = 0
        for j in range(low_bound, upper_bound):
            diff += points_length[(i+j)%len(laser_points)] - points_length[(i+j+1)%len(laser_points)]
        
        curvature = abs(diff)/(upper_bound-low_bound)
        
        if (curvature >= 0.20):
            print(curvature, points_length[i])
            travel_position.append(laser_points[i])

    return travel_position

def find_travel_position1(laser_info, robot_c):
    '''
    use dbscan find the travel position
    '''
    dbscan = DBSCAN(eps=0.3/0.05, min_samples=3)
    laser_points = laser_info['point']
    points_length = laser_info['length']

    filter_points = []
    for i in range(len(laser_points)):
        if (points_length[i] < 0.05) or (points_length[i] > 1.95):
            continue
        filter_points.append(laser_points[i])

    filter_points = np.array(filter_points)
    labels = dbscan.fit_predict(filter_points)
    
    # put different label into different list
    label_list = []
    for i in range(np.max(labels)+1):
        label_list.append([])
    for i in range(len(labels)):
        label_list[labels[i]].append(filter_points[i])
    
    print(np.array(label_list[0]).shape)
    print(np.array(label_list[1]).shape)

    # calculate the center of each lable and get the direction from the robot pos
    center_list = []
    for i in range(len(label_list)):
        center_list.append(np.mean(np.array(label_list[i]), axis=0))

    # calculate the direction and angle of each cluster
    direction_list = []
    direction_angle_list = []
    for i in range(len(center_list)):
        direction_list.append(center_list[i]-robot_c/resolusion)
        direction_angle_list.append(np.arctan2(direction_list[i][1], direction_list[i][0]))
  
    # plot direction_list
    for i in range(len(direction_list)):
        plt.arrow(robot_c[0]/resolusion, robot_c[1]/resolusion, direction_list[i][0], direction_list[i][1], color='tab:blue', width=0.01)

    # calculate the angle between robot_c to the points in each cluster and find the max and min angle and corresponding point
    # min_index min_angle, max_index, max_angle
    max_min_angle = [[0,np.inf,0,-np.inf] for i in range(len(label_list))]

    for i in range(len(label_list)):
        for j in range(len(label_list[i])):
            angle = np.arctan2(label_list[i][j][1]-robot_c[1]/resolusion, label_list[i][j][0]-robot_c[0]/resolusion)
            print('----------------------')
            print('before ',angle, '@@@ ', direction_angle_list[i])
            # calculate the angle between the robot_c to center of the cluster(the angle is betwwen -pi to pi)
            angle = (angle - direction_angle_list[i])
            angle = np.arctan2(np.sin(angle), np.cos(angle))

            print(label_list[i][j], robot_c/resolusion)
            print('after ', angle)
            if angle < max_min_angle[i][1]:
                max_min_angle[i][0] = j
                max_min_angle[i][1] = angle
            if angle > max_min_angle[i][3]:
                max_min_angle[i][2] = j
                max_min_angle[i][3] = angle

    print(max_min_angle)

    # plot the points with max and min angle
    for i in range(len(max_min_angle)):
        plt.plot(label_list[i][max_min_angle[i][0]][0], label_list[i][max_min_angle[i][0]][1], 'ro')
        plt.plot(label_list[i][max_min_angle[i][2]][0], label_list[i][max_min_angle[i][2]][1], 'go')


    plt.scatter(filter_points[:, 0], filter_points[:, 1], c=labels, cmap='viridis', s=60)
    plt.title('DBSCAN Clustering')
    plt.show()




def merge_travel_position(last_position_list, cur_position_list):
    '''
    calculate the relative score and merge the travel position
    '''
    if len(last_position_list) == 0:
        return cur_position_list
    elif len(cur_position_list) == 0:
        return last_position_list
    else:
        score_matrix = np.zeros((len(last_position_list), len(cur_position_list)))
        for i in range(len(last_position_list)):
            for j in range(len(cur_position_list)):
                score_matrix[i, j] = np.linalg.norm(last_position_list[i]-cur_position_list[j])
        score_matrix = score_matrix/np.max(score_matrix)
        score_matrix = 1 - score_matrix
        score_matrix = score_matrix/np.sum(score_matrix, axis=1, keepdims=True)
        score_matrix = np.sum(score_matrix, axis=0)
        score_matrix = score_matrix/np.sum(score_matrix)
        score_matrix = np.argsort(score_matrix)
        score_matrix = score_matrix[::-1]
        print(score_matrix)
        if len(score_matrix) > 3:
            score_matrix = score_matrix[:3]
        return np.array(cur_position_list)[score_matrix]
    

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
    laser_points = None
    travel_position_list = []

    def update_robot_plot(location, re=False):
        global robot_instance
        if not re:
            robot_instance = plt.Circle((location[0]/resolusion, location[1]/resolusion), robot_radius/resolusion, color='tab:gray', fill=False)
        else:
            robot_instance.set_center((location[0]/resolusion, location[1]/resolusion))
        return robot_instance
    
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

    def update_travel_position(ax, position):
        for i in range(len(travel_position_list)):
            # remove the past travel position in the figure
            travel_position_list[i].remove()
               
        travel_position_list.clear()

        for i in range(len(position)):
            temp_position = plt.Circle((position[i][0], position[i][1]), 3.0, color='tab:gray', fill=False)
            travel_position_list.append(temp_position)
            ax.add_patch(temp_position)

        print("len", len(travel_position_list))

    pM.plot_map()
    pM.plot_interpolated_path()
    pM.plot_path()
    ax = plt.gca()
    ax.add_patch(update_robot_plot(start_position*resolusion))
    ax.add_patch(update_laser_points(Path([[0, 0]])))

    obs_points = None
    obs_path = None
    ver_sets = None

    for index in range(len(path)):
        robot_c = path[index]*resolusion
        de_obs = laser.state2obs(robot_c, robot_yaw, False)
        travel_position = find_travel_position1(de_obs, robot_c)
        print(len(travel_position))

        update_robot_plot(robot_c, True)
        update_laser_points(de_obs['point'], True)
        update_travel_position(ax, travel_position)

        plt.legend()
        plt.tight_layout()
        plt.axis('off')
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
        plt.pause(0.1) # pause a bit so that plots are updated
        
        
