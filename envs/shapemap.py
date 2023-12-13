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
import matplotlib.pyplot as plt

def get_H_obs(r) :
    return np.diag([1/r,1/r,0])

def get_H_safe(r, r_safe) :
    return np.diag([1/(r+r_safe), 1/(r+r_safe), 0])

def generate_obstacle_random(num_obs=None, r_safe=0.3, radius_range=[0.5, 1.0], \
                             x_range=[-1.0, 1.0], y_range=[0.7, 5.3], load_mode=True) :
    '''
    generate random circle obstacles
    param: r_safe: safe distance
           num_obs: number of obstacles
           radius_range: range of radius
           x_range: range of x
           y_range: range of y
           load_mode: load the pre-generated obstacles
    '''
    if load_mode:
        obs_data = np.load('envs/maps/circles_obstacles.npz')
        return obs_data['c_list'], obs_data['H_obs_list'], obs_data['H_safe_list'], obs_data['r_list']
    def euclidean_distance(x1, y1, x2, y2):
        '''
        calculate the euclidean distance between two points
        '''
        return math.hypot((x1 - x2), (y1 - y2))

    circle_list = []
    max_iter = 5000

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
        if not any((x2, y2, r2) for x2, y2, r2 in circle_list if euclidean_distance(x, y, x2, y2) < r + r2):
            circle_list.append((x, y, r))  

    c_list = [] # center of circle
    H_obs_list = [] # H matrix for obstacle
    H_safe_list = [] # H matrix for safe distance
    r_list = [] # radius of circle
    for circle in circle_list :
            x,y,r = circle[0],circle[1],circle[2]
            r -= r_safe
            c = np.array([x,y,0])
            c_list.append(c)
            H_obs_list.append(get_H_obs(r))
            H_safe_list.append(get_H_safe(r, r_safe))
            r_list.append(r)
    
    assert len(c_list) == len(H_obs_list)
    assert len(c_list) == len(H_safe_list)
    num_obstacle = len(c)
    
    np.savez_compressed('envs/maps/circles_obstacles.npz', \
                        c_list=c_list, H_obs_list=H_obs_list, H_safe_list=H_safe_list, r_list=r_list)

    return c_list, H_obs_list, H_safe_list, r_list

def test():
    c, H_obs, H_safe, r = generate_obstacle_random()

    robot_c = [0, 3]
    robot_yaw = np.pi
    robot_radius = 0.3
    from sensors.laser import Laser
    laser = Laser(6, beams=64)
    de_obs = laser.state2obs(robot_c, robot_yaw, c, H_obs)
    
    #  Plot 
    ## laser points
    plt.figure(figsize=(5,8))
    ax=plt.gca()
    plt.scatter(de_obs['point'][:, 0], de_obs['point'][:, 1], c='tab:blue', s=2)

    ## plot the first laser line
    first_line_point = laser.first_laser_line(robot_c, robot_yaw)
    plt.plot([robot_c[0], first_line_point[0]], [robot_c[1], first_line_point[1]], c='black', linewidth=2, alpha=0.5)

    ## laser line
    for de in de_obs['point']:
        plt.plot([robot_c[0], de[0]], [robot_c[1], de[1]], c='tab:blue', linewidth=1, alpha=0.5)

    ## obstacle
    for ce,re in zip(c,r) :
        circle1 = plt.Circle((ce[0], ce[1]), re, color='tab:red', alpha=0.5, fill=True)
        circle2 = plt.Circle((ce[0], ce[1]), re+robot_radius, linestyle='--', color='tab:red', alpha=0.5, fill=False)
        ax.add_patch(circle1)
        ax.add_patch(circle2)

    ## ego robot
    dx = np.cos(robot_yaw) * robot_radius
    dy = np.sin(robot_yaw) * robot_radius
    ax.arrow(robot_c[0], robot_c[1], dx, dy, color='tab:green', head_width=0.1, length_includes_head=True)
    robot = plt.Circle((robot_c[0], robot_c[1]), robot_radius, color='tab:gray', fill=False)
    ax.add_patch(robot)

    plt.axis([-2.5, 2.5, -1, 7])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    test()