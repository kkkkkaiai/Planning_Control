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

from copy import deepcopy
from scipy.optimize import minimize

descriptor_resolution = 128
descriptor_angle_range = np.pi*2
descriptor_angle = np.linspace(-np.pi, np.pi, descriptor_resolution)
descriptor_angle_res = descriptor_angle_range/descriptor_resolution
descriptor = [-1 for i in range(descriptor_resolution)]

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
    ### 1 use dbscan to segment the laser points
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
    
    ### 2 get the center of each cluster and calculate the direction from the robot pos
    # put different label into different list
    label_list = []
    for i in range(np.max(labels)+1):
        label_list.append([])
    for i in range(len(labels)):
        label_list[labels[i]].append(filter_points[i])

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

    ### 3. calculate the angle between robot_c to the points in each cluster and find the max and min angle and corresponding point
    # calculate the angle between robot_c to the points in each cluster and find the max and min angle and corresponding point
    # min_index min_angle, max_index, max_angle
    max_min_angle = [[0,np.inf,0,-np.inf] for i in range(len(label_list))]

    for i in range(len(label_list)):
        for j in range(len(label_list[i])):
            angle = np.arctan2(label_list[i][j][1]-robot_c[1]/resolusion, label_list[i][j][0]-robot_c[0]/resolusion)
            # print('----------------------')
            # print('before ',angle, '@@@ ', direction_angle_list[i])
            # calculate the angle between the robot_c to center of the cluster(the angle is betwwen -pi to pi)
            angle = (angle - direction_angle_list[i])
            angle = np.arctan2(np.sin(angle), np.cos(angle))

            # print(label_list[i][j], robot_c/resolusion)
            # print('after ', angle)
            if angle < max_min_angle[i][1]:
                max_min_angle[i][0] = j
                max_min_angle[i][1] = angle
            if angle > max_min_angle[i][3]:
                max_min_angle[i][2] = j
                max_min_angle[i][3] = angle

    # record the travel position in each cluster
    travel_position = []
    for i in range(len(max_min_angle)):
        travel_position.append(label_list[i][max_min_angle[i][0]])
        travel_position.append(label_list[i][max_min_angle[i][2]])

    # plot the points with max and min angle
    for i in range(len(max_min_angle)):
        plt.scatter(label_list[i][max_min_angle[i][0]][0], label_list[i][max_min_angle[i][0]][1], c='tab:red', s=60)
        plt.scatter(label_list[i][max_min_angle[i][2]][0], label_list[i][max_min_angle[i][2]][1], c='tab:red', s=60)

    ### 4.get the minimum radius circle from the robot positon to the nearst laser points
    # the minimum radius circle from the robot positon to the nearst laser points
    min_radius = np.inf
    min_radius_index = 0
    for i in range(len(filter_points)):
        radius = np.linalg.norm(filter_points[i]-robot_c/resolusion)
        if radius < min_radius:
            min_radius = radius
            min_radius_index = i

    # plot the minimum radius circle
    circle = plt.Circle((robot_c[0]/resolusion, robot_c[1]/resolusion), min_radius, color='tab:gray', fill=False) 
    ax = plt.gca()
    ax.add_patch(circle)

    ### 5.calculate the tangent point and the tangent line, it is used to filter the points and determine which points
    ### could be used for constructing the descriptor  
    # for the travel points, calculate the outer tangent point and the angle between the point and travel point
    tangent_point_and_length = []
    for i in range(len(travel_position)):
        distance = np.linalg.norm(travel_position[i]-robot_c/resolusion)
        # calculate the distance between the robot_c and the tangent point
        length = np.sqrt(distance**2 - min_radius**2)
        # unit vector between the point and robot_c
        unit_vector = (robot_c/resolusion - travel_position[i])/distance
        # angle between the tangent line and the line between the robot_c and the travel point
        angle = np.arcsin(min_radius/distance)
        # calculate the tangent point 
        x1 = unit_vector[0]*np.cos(angle) - unit_vector[1]*np.sin(angle)
        y1 = unit_vector[0]*np.sin(angle) + unit_vector[1]*np.cos(angle)
        x2 = unit_vector[0]*np.cos(-angle) - unit_vector[1]*np.sin(-angle)
        y2 = unit_vector[0]*np.sin(-angle) + unit_vector[1]*np.cos(-angle)
        
        x1 = x1 * length
        x2 = x2 * length
        y1 = y1 * length
        y2 = y2 * length

        if robot_c[0] < travel_position[i][0]:
            x1 = x1
            x2 = x2
        if robot_c[1] < travel_position[i][1]:
            y1 = y1
            y2 = y2

        x1 += travel_position[i][0]
        x2 += travel_position[i][0]
        y1 += travel_position[i][1]
        y2 += travel_position[i][1]

        tangent_point_and_length.append([x1, y1, x2, y2, length])

        # plot the tangent point
        plt.scatter(x1, y1, c='tab:blue', s=60)
        plt.scatter(x2, y2, c='tab:blue', s=60)

        # plot the tangent line from travel point to the tangent point
        plt.plot([travel_position[i][0], x1], [travel_position[i][1], y1], c='tab:blue', linewidth=1, alpha=0.5)
        plt.plot([travel_position[i][0], x2], [travel_position[i][1], y2], c='tab:blue', linewidth=1, alpha=0.5)

    ### 6. calculate the descriptor
    # calculate the descriptor
    descriptor_list = []
    for i in range(len(travel_position)):
        temp_descriptor = deepcopy(descriptor)
        min_angle = np.arctan2(tangent_point_and_length[i][1]-travel_position[i][1], tangent_point_and_length[i][0]-travel_position[i][0])
        max_angle = np.arctan2(tangent_point_and_length[i][3]-travel_position[i][1], tangent_point_and_length[i][2]-travel_position[i][0])
        if min_angle > max_angle:
            temp_angle = min_angle
            min_angle = max_angle
            max_angle = temp_angle

        # calculate the angle of the travel_point to the robot_c
        choose_mode = 0
        if min_angle * max_angle > 0:
            choose_mode = 1

        # [DEBUG]
        # print("=================")
        # print("angle info",  min_angle, max_angle)
        
        for j in range(len(filter_points)):
            ## if the distance between the travel point and the laser point is less than the length in tangent_point_and_length
            ## the point is not used for constructing the descriptor
            distance = np.linalg.norm(travel_position[i]-robot_c/resolusion)
            # choice 1
            if np.linalg.norm(filter_points[j]-travel_position[i]) < distance:
            # choice 2
            # if np.linalg.norm(filter_points[j]-travel_position[i]) < tangent_point_and_length[i][4]:
                continue
            
            ## if the angle between the travel point and the laser point is out of the range of 
            ## the angle between the tangent line, it is not used for constructing the descriptor
            angle = np.arctan2(filter_points[j][1]-travel_position[i][1], filter_points[j][0]-travel_position[i][0])
            if choose_mode == 1:
                if angle < min_angle or angle > max_angle:
                    continue
            else:
                if angle > min_angle and angle < max_angle:
                    continue
            # [DEBUG]
            # print(angle)
            ## calculate the index of the descriptor
            index = int((angle+np.pi)/descriptor_angle_res)%descriptor_resolution

            temp_descriptor[index] = np.linalg.norm(filter_points[j]-travel_position[i])
        
        descriptor_list.append(temp_descriptor)

    ### 7.construct transfer point
    # transfer point connect the robot_c and the travel points in each cluster
    transfer_point = []
    weights = [1, 1, 1]

    def weighted_distance(x, points, weights):
        return sum(weights[i] * np.linalg.norm(x - p) for i, p in enumerate(points))

    for i in range(0, len(travel_position), 2):
        points = np.array([robot_c/resolusion, travel_position[i], travel_position[i+1]])
        # calculate the distance between the travel_positions
        distance = np.linalg.norm(travel_position[i]-travel_position[i+1])
        robot_distance = np.linalg.norm(travel_position[i]-robot_c/resolusion)
        print(robot_distance, distance, robot_distance/distance)
        temp_weights = deepcopy(weights)
        temp_weights[0] = min(robot_distance/distance, 1.85)
        # temp_weights[1] = distance
        # temp_weights[2] = distance
        # do normalization
        temp_weights = temp_weights/np.sum(temp_weights)

        initial_guess = np.mean(points, axis=0)
        result = minimize(lambda x: weighted_distance(x, points, temp_weights), initial_guess, method='L-BFGS-B')
        optimal_point = result.x
        # [DEBUG]
        # print(initial_guess)
        transfer_point.append(optimal_point)
        print(points, optimal_point)
        # plot the transfer point
        plt.scatter(optimal_point[0], optimal_point[1], c='tab:green', s=60)
        # # draw the line between the transfer point and the robot_c and the travel point
        plt.plot([robot_c[0]/resolusion, optimal_point[0]], [robot_c[1]/resolusion, optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)
        plt.plot([travel_position[i][0], optimal_point[0]], [travel_position[i][1], optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)
        plt.plot([travel_position[i+1][0], optimal_point[0]], [travel_position[i+1][1], optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)

    plt.scatter(filter_points[:, 0], filter_points[:, 1], c=labels, cmap='viridis', s=60)
    plt.title('DBSCAN Clustering')

    # [debug]plot one the descriptor
    descriptor_index = 0
    if len(travel_position) > 0:
        plot_the_descriptor(travel_position[descriptor_index], descriptor_list[descriptor_index])
    
    plt.show()



def plot_the_descriptor(descriptor_pos, descriptor):
    points = []
    # print(descriptor_pos)
    # print(descriptor)
    for i in range(len(descriptor)):
        if descriptor[i] != -1:
            points.append([descriptor_pos[0]+descriptor[i]*np.cos(descriptor_angle[i]), \
                           descriptor_pos[1]+descriptor[i]*np.sin(descriptor_angle[i])])

    if len(points) == 0:
        return
    ax.scatter(np.array(points)[:, 0], np.array(points)[:, 1], c='tab:orange', s=60)



def get_tangent_line(tx, ty, ox, oy, r):
    distance = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
    length = math.sqrt(distance ** 2 - r ** 2)

    if distance <= r:
        print("输入的数值不在范围内")
        return

    cx = abs(ox - tx) / distance
    cy = abs(oy - ty) / distance

    angle = math.asin(r / distance)

    q1x = cx * math.cos(angle) - cy * math.sin(angle)
    q1y = cx * math.sin(angle) + cy * math.cos(angle)
    q2x = cx * math.cos(-angle) - cy * math.sin(-angle)
    q2y = cx * math.sin(-angle) + cy * math.cos(-angle)

    q1x = q1x * length
    q1y = q1y * length
    q2x = q2x * length
    q2y = q2y * length

    if ox < tx:
        q1x = -q1x 
        q2x = -q2x
    
    if oy < ty:
        q1y = -q1y 
        q2y = -q2y

    q1x += tx
    q2x += tx
    q1y += ty
    q2y += ty

    return [q1x, q1y, q2x, q2y]

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
    start_position = np.array([0.5, 5.0])  # wrong tangent point
    # start_position = np.array([1.0, 2.0])  # wrong tangent point
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
        
        
