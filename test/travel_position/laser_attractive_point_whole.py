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
from path_manager import PathManager
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

resolution = 0.05

id_to_node = {}
all_travel_position = []
current_travel_position_id = None
last_travel_position_id = None
global_root_id = None

class IDAssigner:
    def __init__(self, number = 1000):
        self._id_max = number
        self._id_list = [-1] * number
        self._id_num = 0

    def get_id(self):
        if self._id_num == self._id_max:
            # expand the list
            self._id_max *= 2
            self._id_list.extend([-1] * self._id_max)

        for i in range(len(self._id_list)):
            if self._id_list[i] == -1:
                self._id_list[i] = 1
                self._id_num += 1
                return i
    
    def remove_id(self, id_number):
        self._id_list[id_number] = -1
        self._id_num -= 1

id_assigner = IDAssigner()

class TravelPositionNode:
    def __init__(self) -> None:
        self._position = None
        self._descriptor = None
        self._id = None
        self._stuck = False
        self._free_direction = None
        self._next_transfer = []
        self._last_transfer = []
        self._fscore = 0
        self._gscore = 0


# key point 1: find the travel position
def find_travel_position(laser_info, robot_c):
    '''
    use dbscan find the travel position
    '''
    ### 1.use dbscan to segment the laser points
    dbscan = DBSCAN(eps=0.35/resolution, min_samples=2)
    laser_points = laser_info['point']
    points_length = laser_info['length']

    filter_points = []
    for i in range(len(laser_points)):
        # if (points_length[i] < 0.05) or (points_length[i] > 2.0):
        #     continue
        filter_points.append(laser_points[i])

    filter_points = np.array(filter_points)
    labels = dbscan.fit_predict(filter_points)
    
    ### 2.get the center of each cluster and calculate the direction from the robot pos
    # put different label into different list
    label_list = []
    for i in range(np.max(labels)+1):
        label_list.append([])
    for i in range(len(labels)):
        if labels[i] != -1:
            label_list[labels[i]].append(filter_points[i])

    # calculate the center of each lable and get the direction from the robot pos
    center_list = []
    for i in range(len(label_list)):
        center_list.append(np.mean(np.array(label_list[i]), axis=0))

    # calculate the direction and angle of each cluster
    direction_list = []
    direction_angle_list = []
    for i in range(len(center_list)):
        direction_list.append(center_list[i]-robot_c)
        direction_angle_list.append(np.arctan2(direction_list[i][1], direction_list[i][0]))
  
    # plot direction_list
    # [DISPLAY]
    # for i in range(len(direction_list)):
    #     plt.arrow(robot_c[0], robot_c[1], direction_list[i][0], direction_list[i][1], color='tab:blue', width=0.01)

    ### 3.calculate the angle between robot_c to the points in each cluster and find the max and min angle and corresponding point
    # calculate the angle between robot_c to the points in each cluster and find the max and min angle and corresponding point
    # min_index min_angle, max_index, max_angle
    max_min_angle = [[0,np.inf,0,-np.inf] for i in range(len(label_list))]

    for i in range(len(label_list)):
        for j in range(len(label_list[i])):
            angle = np.arctan2(label_list[i][j][1]-robot_c[1], label_list[i][j][0]-robot_c[0])
            # print('----------------------')
            # print('before ',angle, '@@@ ', direction_angle_list[i])
            # calculate the angle between the robot_c to center of the cluster(the angle is betwwen -pi to pi)
            angle = (angle - direction_angle_list[i])
            angle = np.arctan2(np.sin(angle), np.cos(angle))

            # print(label_list[i][j], robot_c)
            # print('after ', angle)
            if angle < max_min_angle[i][1]:
                max_min_angle[i][0] = j
                max_min_angle[i][1] = angle
            if angle > max_min_angle[i][3]:
                max_min_angle[i][2] = j
                max_min_angle[i][3] = angle

    # record the travel position in each cluster
    travel_position = []
    free_direction_angle = []
    for i in range(len(max_min_angle)):
        # check the distance whether is close to the max laser range
        if np.linalg.norm(label_list[i][max_min_angle[i][0]]-robot_c) > 3.7/resolution or \
           np.linalg.norm(label_list[i][max_min_angle[i][2]]-robot_c) > 3.7/resolution:
            continue

        # calculate the free_direction angle
        free_direction_angle.append(np.arctan2(label_list[i][max_min_angle[i][0]][1]-label_list[i][max_min_angle[i][2]][1], \
                                        label_list[i][max_min_angle[i][0]][0]-label_list[i][max_min_angle[i][2]][0]))
        travel_position.append(label_list[i][max_min_angle[i][0]])
        free_direction_angle.append(np.arctan2(label_list[i][max_min_angle[i][2]][1]-label_list[i][max_min_angle[i][0]][1], \
                                        label_list[i][max_min_angle[i][2]][0]-label_list[i][max_min_angle[i][0]][0]))
        travel_position.append(label_list[i][max_min_angle[i][2]])

    # plot the points with max and min angle
    # [DISPLAY]
    # for i in range(len(max_min_angle)):
    #     plt.scatter(label_list[i][max_min_angle[i][0]][0], label_list[i][max_min_angle[i][0]][1], c='tab:red', s=15)
    #     plt.scatter(label_list[i][max_min_angle[i][2]][0], label_list[i][max_min_angle[i][2]][1], c='tab:red', s=15)

    ### 4.get the minimum radius circle from the robot positon to the nearest laser points
    # the minimum radius circle from the robot positon to the nearest laser points
    min_radius = np.inf
    min_radius_index = 0
    for i in range(len(filter_points)):
        radius = np.linalg.norm(filter_points[i]-robot_c)
        if radius < min_radius:
            min_radius = radius
            min_radius_index = i

    # plot the minimum radius circle
    circle = plt.Circle((robot_c[0], robot_c[1]), min_radius, color='tab:gray', fill=False) 
    ax = plt.gca()
    ax.add_patch(circle)

    ### 5.calculate the tangent point and the tangent line, it is used to filter the points and determine which points
    ### could be used for constructing the descriptor  
    # for the travel points, calculate the outer tangent point and the angle between the point and travel point
    tangent_point_and_length = []
    for i in range(len(travel_position)):
        distance = np.linalg.norm(travel_position[i]-robot_c)
        # calculate the distance between the robot_c and the tangent point
        length = np.sqrt(distance**2 - min_radius**2)
        # unit vector between the point and robot_c
        unit_vector = (robot_c - travel_position[i])/distance
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
        plt.scatter(x1, y1, c='tab:blue', s=15)
        plt.scatter(x2, y2, c='tab:blue', s=15)

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
            distance = np.linalg.norm(travel_position[i]-robot_c)
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
        points = np.array([robot_c, travel_position[i], travel_position[i+1]])
        # calculate the distance between the travel_positions
        distance = np.linalg.norm(travel_position[i]-travel_position[i+1])
        robot_distance = np.linalg.norm(travel_position[i]-robot_c)
        temp_weights = deepcopy(weights)
        temp_weights[0] = min(1.5*robot_distance/distance, 1.85)
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
        # print(points, optimal_point)
        # plot the transfer point
        plt.scatter(optimal_point[0], optimal_point[1], c='tab:green', s=15)
        # # draw the line between the transfer point and the robot_c and the travel point
        plt.plot([robot_c[0], optimal_point[0]], [robot_c[1], optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)
        plt.plot([travel_position[i][0], optimal_point[0]], [travel_position[i][1], optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)
        plt.plot([travel_position[i+1][0], optimal_point[0]], [travel_position[i+1][1], optimal_point[1]], c='tab:green', linewidth=1, alpha=0.5)

    ### 8. save the travel position and the descriptor
    travel_position_list = []
    
    for i in range(len(travel_position)):
        temp_node = TravelPositionNode()
        temp_node._position = travel_position[i]
        temp_node._descriptor = descriptor_list[i]
        temp_node._id = -1
        # calculate the direction based on the angle between the travel point and the center of the cluster
        temp_node._free_direction = free_direction_angle[i]
        temp_node._next_transfer = []
        temp_node._last_transfer = []

        travel_position_list.append(temp_node)

    # plt.scatter(filter_points[:, 0], filter_points[:, 1], c=labels, cmap='viridis', s=30)
    plt.title('Travel Position')

    # [debug]plot one the descriptor
    # descriptor_index = 0
    # if len(travel_position) > 0:
    #     plot_the_descriptor(travel_position[descriptor_index], descriptor_list[descriptor_index])

    return travel_position_list

# key point2: merge the travel position in different frame
def merge_travel_position(new_travel_position, travel_positions_list):
    '''
    new_travel_position: the travel position in current frame
    travel_positions_list: the travel position in all past frame
    return : travel_positions_list
             new_travel_position_list
    '''
    new_travel_position_list = []

    if len(travel_positions_list) == 0:
        for i in range(len(new_travel_position)):
            # collect new travel position
            new_travel_position[i]._id = id_assigner.get_id()
            id_to_node[new_travel_position[i]._id] = new_travel_position[i]
            new_travel_position_list.append(new_travel_position[i]._id)
        return new_travel_position, new_travel_position_list
    
    # determine whether it is the same point based on two criteria
    # 1. the distance between the new travel position and the past travel position
    # 2. the descriptor of the new travel position and the past travel position
    delete_list = []
    for i in range(len(travel_positions_list)):
        position_in_all = travel_positions_list[i]._position
        for j in range(len(new_travel_position)):
            # judge the distance first
            position_new = new_travel_position[j]._position
            
            # check the distance
            if np.linalg.norm(position_new-position_in_all) < 0.3/resolution:
                # method 1
                # descriptor_in_all = travel_positions_list[i]._descriptor
                # descriptor_new = new_travel_position[j]._descriptor
                # if is_same_descriptor(descriptor_new, descriptor_in_all):
                #     travel_positions_list[i]._descriptor = merge_descriptor(position_in_all, position_new,
                #                                                             descriptor_in_all, descriptor_new)
                #     # if merge the descriptor, record the index of the new travel position
                #     delete_list.append(j)

                # method 2
                delete_list.append(j)
                    
    # add new_descriptor in the travel position list
    for i in range(len(new_travel_position)):
        if i not in delete_list:
            new_travel_position[i]._id = id_assigner.get_id()
            travel_positions_list.append(new_travel_position[i])
            # collect new travel position
            new_travel_position_list.append(new_travel_position[i]._id)
            id_to_node[new_travel_position[i]._id] = new_travel_position[i]

    return travel_positions_list, new_travel_position_list


def is_same_descriptor(descriptor1, descriptor2):
    # judge whether the two descriptor is the same
    # the first step is to calculate the scope of the updated data
    # If the range overlap is relatively high, calculate the difference between the overlap data
    # if the overlap is relatively low, directily updating the descriptor
    overlap_value = 0
    overlap_count = 0
    for i in range(len(descriptor1)):
        if descriptor1[i] != -1 and descriptor2[i] != -1:
            diff = abs(descriptor1[i] - descriptor2[i])
            overlap_count += 1
            if diff < 0.2:
                overlap_value += 1
    overlap_value = overlap_value/overlap_count
    if overlap_count < len(descriptor1)*0.15 or overlap_value > 0.5:
        return True

    return False

def merge_descriptor(position1, position2, descriptor1, descriptor2):
    # merge the descriptor
    # assign the position2's descriptor to position1
    # use vector addition to merge the descriptor
    for i in range(len(descriptor2)):
        # calculate the position of the descriptor2 
        x = position2[0] + descriptor2[i]*np.cos(descriptor_angle[i])
        y = position2[1] + descriptor2[i]*np.sin(descriptor_angle[i])

        # calculate the vector between the position2 and current index position of the descriptor 
        vector = np.array([x-position1[0], y-position1[1]])

        # calculate the angle between the vector and the x axis
        angle = np.arctan2(vector[1], vector[0])

        # calculate the index of the descriptor
        index = int((angle+np.pi)/descriptor_angle_res)%descriptor_resolution

        # calculate the distance
        distance = np.linalg.norm(vector)

        # update the descriptor
        descriptor1[index] = distance

    return descriptor1


def iplanning(de_obs, robot_c, goal_position):
    global all_travel_position, current_travel_position_id, last_travel_position_id, global_root_id
    next_travel_position = None
    travel_position_list = find_travel_position(de_obs, robot_c)

    all_travel_position, new_travel_position = merge_travel_position(travel_position_list, all_travel_position)

    if current_travel_position_id is None:
        temp_node = TravelPositionNode()
        temp_node._position = robot_c
        temp_node._descriptor = None
        temp_node._id = id_assigner.get_id()
        global_root_id = temp_node._id
        temp_node._next_transfer = []
        temp_node._last_transfer = []
        current_travel_position_id = temp_node._id
        last_travel_position_id = temp_node._id

        all_travel_position.append(temp_node)
        id_to_node[temp_node._id] = deepcopy(temp_node)

    # record the relationship between the id and the travel position node

    if len(new_travel_position) > 0:
        construct_graph(current_travel_position_id, last_travel_position_id, new_travel_position, robot_c)


    next_travel_position = find_travel_position_to_goal(all_travel_position, travel_position_list, robot_c, goal_position)
    print('after find ', next_travel_position._id)
    # judge next_travel_position whether is the same as current_travel_position
    if next_travel_position is not None:
        # if the distance between the robot_c and the next_travel_position is less than 0.3m,
        # and the next_travel_position is not the same as the current_travel_position
        if np.linalg.norm(next_travel_position._position - robot_c) > 0.3/resolution and \
            next_travel_position._id != current_travel_position_id:
            print('in check ', current_travel_position_id, next_travel_position._id)
            last_travel_position_id = deepcopy(current_travel_position_id)
            current_travel_position_id = deepcopy(next_travel_position._id)

    print('in planning', current_travel_position_id, next_travel_position._id)

    return next_travel_position


def find_travel_position_to_goal(all_travel_position, travel_position_list, robot_c, goal_position):
    '''
    nearest travel position to the goal
    '''
    # add current position and goal_position to the travel position list temporarily
    # temp_start_node = TravelPositionNode()
    # temp_start_node._position = robot_c
    # temp_start_node._descriptor = None
    # temp_start_node._id = -2

    # temp_goal_node = TravelPositionNode()
    # temp_goal_node._position = goal_position
    # temp_goal_node._descriptor = None
    # temp_goal_node._id = -2

    # add the id with the travel position list in current frame to the next_transfer, and calculate the fscore and gscore
    # for i in range(len(travel_position_list)):
    #     temp_start_node._next_transfer.append(travel_position_list[i]._id)
        # fscore is the distance from the travel position to the current position
        # travel_position_list[i]._fscore = np.linalg.norm(travel_position_list[i]._position - robot_c)
        # gscore is the distance from the travel position to the goal position
        # travel_position_list[i]._gscore = np.linalg.norm(travel_position_list[i]._position - goal_position)
    
    # find the nearest travel position to the goal and construct connection between the travel position and the goal
    min_goal_distance = np.inf
    min_index = -1
    print('global_root_id', global_root_id)
    for i in range(len(all_travel_position)):
        if all_travel_position[i]._id == global_root_id:
            continue
        g_distance = 0.9*np.linalg.norm(all_travel_position[i]._position - goal_position)
        f_distance = 1*np.linalg.norm(all_travel_position[i]._position - robot_c)
        distance = g_distance
        print('index', i, distance)
        if distance < min_goal_distance:
            min_goal_distance = distance
            min_index = i

    # travel_position_list[min_index]._next_transfer.append(temp_goal_node._id)

    path_list = []
    final_travel_position = all_travel_position[min_index]
    print('final_travel_position_id', final_travel_position._id)
    # # use the Breadth first search to find the nearest travel position to the goal
    # open_list = []
    # close_list = []
    # open_list.append(temp_start_node)

    # # when the open list is not empty or the goal is found in the open list
    # while len(open_list) > 0:
    #     # find the travel position with the minimum fscore
    #     min_fscore = np.inf
    #     min_index = 0
    #     for i in range(len(open_list)):
    #         if open_list[i]._fscore < min_fscore:
    #             min_fscore = open_list[i]._fscore
    #             min_index = i
        
    #     # pop the travel position with the minimum fscore
    #     final_travel_position = open_list.pop(min_index)
    #     close_list.append(final_travel_position)

    #     # check whether the current travel position is the goal
    #     if final_travel_position._position[0] == goal_position[0] and \
    #        final_travel_position._position[1] == goal_position[1]:
    #         break

    #     # add the next transfer to the open list
    #     for i in range(len(final_travel_position._next_transfer)):
    #         next_travel_position = id_to_node[final_travel_position._next_transfer[i]]
    #         next_travel_position._fscore = final_travel_position._fscore + \
    #                                         np.linalg.norm(next_travel_position._position - final_travel_position._position)
    #         next_travel_position._gscore = np.linalg.norm(next_travel_position._position - goal_position)
    #         open_list.append(next_travel_position)

    # retrive the path from the final travel position
    # add the travel position to the path list
    # while len(final_travel_position._last_transfer) > 0:
    #     path_list.append(final_travel_position)
    #     final_travel_position = id_to_node[final_travel_position._last_transfer[0]]

    # if the length of the path is less than 2, return None
    # if len(path_list) < 1:
    #     return None
    # else:
    #     # reverse the path list
    #     path_list.reverse()
    #     # return the second travel position in the path list
    return final_travel_position


def plot_graph(all_travel_position):
    # plot the graph
    for i in range(len(all_travel_position)):
        current_position = all_travel_position[i]._position
        for k in range(len(all_travel_position[i]._next_transfer)):
            next_position = id_to_node[all_travel_position[i]._next_transfer[k]]
            # print('current position', current_position, next_position._position)
            plt.plot([current_position[0], next_position._position[0]], [current_position[1], next_position._position[1]], c='tab:blue', linewidth=1, alpha=0.5)
        
        id_list = all_travel_position[i]._last_transfer
        for k in range(len(all_travel_position[i]._last_transfer)):
            last_position = id_to_node[all_travel_position[i]._last_transfer[k]]
            # print('last position', current_position, last_position._position)
            plt.plot([current_position[0], last_position._position[0]], [current_position[1], last_position._position[1]], c='tab:blue', linewidth=1, alpha=0.5)

def construct_graph(current_travel_position_id, last_tp_id, new_travel_position, robot_c):
    global last_travel_position_id
    # if the distance between the current travel position is less than 1.0, 
    # add connection between the current travel position and the new travel position
    print("[construct graph - new] ", len(new_travel_position), " ", new_travel_position)

    # if np.linalg.norm(current_travel_position._position - last_travel_position._position) < 1.0:
    #     for i in range(len(new_travel_position)):
    #         id_to_node[current_travel_position._id]._next_transfer.append(new_travel_position[i])
    #         id_to_node[new_travel_position[i]]._last_transfer.append(current_travel_position._id)
    #     last_travel_position = id_to_node[current_travel_position._id]
    # else:
    #     # add a new travel position between the last travel position and the current travel position, 
    #     # then updating the connection between the last travel position and the current travel position with the new travel position
    #     # 1.delete the connection to current travel position in last_travel_position's next_transfer
    #     if last_travel_position is not None:
    #         for i in range(len(last_travel_position._next_transfer)):
    #             if last_travel_position._next_transfer[i] == current_travel_position._id:
    #                 last_travel_position._next_transfer.pop(i)
    #                 break
            
    #         for i in range(len(current_travel_position._last_transfer)):
    #             if current_travel_position._last_transfer[i] == last_travel_position._id:
    #                 current_travel_position._last_transfer.pop(i)
    #                 break

    print('current and last travel_position_id', current_travel_position_id, last_tp_id)
    # 2.instanciate a new travel position
    last_travel_position = id_to_node[last_tp_id]
    if np.linalg.norm(last_travel_position._position - robot_c) > 0.05/0.05:
    # if True:
        curr_id = current_travel_position_id
        current_node = id_to_node[curr_id]

        last_id = last_tp_id
        last_node = id_to_node[last_id]

        for i in range(len(last_node._next_transfer)):
            if last_node._next_transfer[i] == current_node._id:
                last_node._next_transfer.pop(i)
                break
        for i in range(len(current_node._last_transfer)):
            if current_node._last_transfer[i] == last_node._id:
                current_node._last_transfer.pop(i)
                break

        temp_node = TravelPositionNode()
        temp_node._position = robot_c
        temp_node._descriptor = None
        temp_node._id = id_assigner.get_id()
        
        # add the connection with the sequence, last_node , new_temp_node, current_node
        # take care !!!
        last_node._next_transfer.append(temp_node._id)
        temp_node._last_transfer = [last_node._id]
        
        temp_node._next_transfer = [current_node._id]
        current_node._last_transfer = [temp_node._id]
        # add the connection to current travel position in new_travel_position's next_transfer
        for i in range(len(new_travel_position)):
            temp_node._next_transfer.append(new_travel_position[i])
            id_to_node[new_travel_position[i]]._last_transfer.append(temp_node._id)

        # 3.add the new travel position to the all travel position list
        all_travel_position.append(temp_node)
        id_to_node[temp_node._id] = temp_node

        last_travel_position_id = temp_node._id    
    else:
        for i in range(len(new_travel_position)):
            id_to_node[last_travel_position._id]._next_transfer.append(new_travel_position[i])
            id_to_node[new_travel_position[i]]._last_transfer.append(last_travel_position._id)


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
    ax.scatter(np.array(points)[:, 0], np.array(points)[:, 1], c='tab:orange', s=15)


if __name__ == '__main__':
    map_file='envs/maps/obstacle_map_occupancy.png'
    gridmap = OccupancyGridMap.from_png(map_file, resolution)
    pM = PathManager(gridmap)
    start_position = np.array([1.0, 2.0])  # wrong tangent point
    # start_position = np.array([1.0, 2.0])  # wrong tangent point
    end_position = np.array([4, 6.0])
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

    iter = 100
    index = 0
    robot_c = start_position
    move_distance = 5*resolution

    achive_goal = False

    while index < iter:
        print("INDEX", index, robot_c)
        de_obs = laser.state2obs(robot_c, robot_yaw, False)
        travel_position = iplanning(de_obs, robot_c/resolution, end_position/resolution)

        current_position = robot_c
        # get the position along the free direction
        if travel_position._free_direction is not None:
            next_position = travel_position._position + np.array([np.cos(travel_position._free_direction), np.sin(travel_position._free_direction)])*0.3/resolution
        else:
            next_position = travel_position._position

        next_position = deepcopy(next_position)
        next_position *= resolution
        # move along direction with fixed move distance 
        print('position ', current_position, next_position)
        direction = next_position - current_position
        direction_angle = np.arctan2(direction[1], direction[0])
        if np.linalg.norm(direction) < 0.00001:
            robot_c = current_position
        else:
            direction = direction / np.linalg.norm(direction)
            robot_c = current_position + direction * move_distance

        # check if the robot is close to the goal
        if np.linalg.norm(robot_c/resolution - end_position/resolution) < 0.1/resolution:
            achive_goal = True
            break

        ax.add_patch(update_robot_plot(start_position*resolution))
        ax.add_patch(update_laser_points(Path([[0, 0]])))

        update_robot_plot(robot_c, True)
        update_laser_points(de_obs['point'], True)

        pM.plot_map()
        pM.plot_interpolated_path()
        pM.plot_path()
        
        plt.tight_layout()
        # save path figure with dpi 300
        plt.savefig('test/travel_position/result/{}.png'.format(index), dpi=600)
       
        # 2. save travel position figure
        # fig = plt.figure()
        # pM.plot_map()
        # # plot all travel position in another figure
        # for i in range(len(all_travel_position)):
        #     plt.scatter(all_travel_position[i]._position[0], all_travel_position[i]._position[1], c='tab:blue', s=5)
        # plt.savefig('test/travel_position/result/travel_position_{}.png'.format(index), dpi=600)

        # plot the graph
        fig = plt.figure()
        pM.plot_map()
        plot_graph(all_travel_position)
        # plot the robot's position
        plt.scatter(robot_c[0]/resolution, robot_c[1]/resolution, c='tab:green', s=10)
        # plot the goal's position
        plt.scatter(end_position[0]/resolution, end_position[1]/resolution, c='tab:red', s=10)
        # plot the travel position
        if travel_position is not None:
            plt.scatter(next_position[0]/resolution, next_position[1]/resolution, c='tab:red', s=20)
        plt.savefig('test/travel_position/result/graph_{}.png'.format(index), dpi=600)

        plt.cla()

        index += 1 
        
