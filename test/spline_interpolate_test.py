import sys, os
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from planner.trajectory_generator.spline_interpolate import *
from planner.path_generator.path import *
from planner.path_generator.path_manager import PathManager
from models.python.geometry_utils import *
from models.python.differential_car import DifferentialCarSystem
from sensors.laser_anyshape import Laser
from envs.gridmap import OccupancyGridMap
from controller.python.pid import PIDController

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches

resolution = 0.05

def get_ref_target_point(position, ref_path, dt=0.1, d_dist=1.0):
    ind, _ = ref_path.nearest_index(position)
    if (np.linalg.norm(position[0:2]-np.array([ref_path.cx[ind], ref_path.cy[ind]])) < d_dist):
        ind += 1

    target_ref = np.zeros(3)
    target_ref[0] = ref_path.cx[ind]
    target_ref[1] = ref_path.cy[ind]
    target_ref[2] = ref_path.cyaw[ind]

    return target_ref

class Robot:
    def __init__(self, robot_pos=None, robot_theta=None, robot_shape=None, resolution=None):
        self.resolution = resolution
        self.last_velocity = 0
        self.robot_representation(robot_pos, robot_theta, robot_shape)
        self.gen_occupancy_type()

    def robot_representation(self, robot_pos=None, robot_theta=None, robot_shape=None):
        if robot_pos.any():
            self.robot_center_pos = self.last_center_pos = robot_pos
        else:
            self.robot_center_pos = self.last_center_pos = np.array([0, 0])

        if robot_theta:
            self.robot_theta = np.radians(robot_theta)
        else:
            self.robot_theta = 0

        if robot_shape:
            self.robot_height = robot_shape[1]
            self.robot_width = robot_shape[0]
        else:
            self.robot_width = 0.5
            self.robot_height = 0.3

        self.T_c_to_c_matrix = np.array([[1, 0, self.robot_width/2], [0, 1, self.robot_height/2], [0, 0, 1]])
        
        self.robot_patch = patches.Rectangle(self.get_patch_center()/self.resolution, self.robot_width/self.resolution, self.robot_height/self.resolution, 
                                             np.rad2deg(self.robot_theta), color='tab:gray', fill=False)

    def get_patch_center(self):
        # get the cornor coordinate according to the center position
        current_T_matrix = np.array([[np.cos(self.robot_theta), -np.sin(self.robot_theta), self.robot_center_pos[0]], 
                                     [np.sin(self.robot_theta), np.cos(self.robot_theta), self.robot_center_pos[1]], 
                                     [0, 0, 1]])
        return np.matmul(current_T_matrix, np.linalg.inv(self.T_c_to_c_matrix))[0:2, 2]

    def gen_occupancy_type(self):
        # robot is modeled as a superellipse
        self.occ_manager = SuperEllipse([self.robot_width*0.55, self.robot_height*0.6], 0.35, self.robot_center_pos, self.robot_theta, 20)

    def update_state(self, state, theta, plot=False, ax=None, re=False):
        self.last_center_pos = self.robot_center_pos
        self.robot_center_pos = np.array(state)
        self.robot_theta = theta
        self.occ_manager.update_state(state, theta)

        if plot:
            self.plot(ax, re)

    def update_integrate_state(self, input, dt=0.1, plot=False, ax=None, re=False, type='differential'): # type can be ackermann or differential
        # input is a tuple of (v, w)        
        if type == 'differential':
            robot_pos = self.robot_center_pos + np.array([input[0]*np.cos(self.robot_theta), input[0]*np.sin(self.robot_theta)])*dt
            robot_theta = self.robot_theta + input[1]*dt
            if robot_theta > 2 * math.pi:
                robot_theta -= 2*math.pi

        self.update_state(robot_pos, robot_theta, plot=plot, ax=ax, re=re)

    def get_transaltion(self):
        return self.robot_center_pos

    def get_rotation(self):
        return self.robot_theta
    
    def get_velocity(self, dt=-0.1):
        dx = self.robot_center_pos[0] - self.last_center_pos[0]
        dy = self.robot_center_pos[1] - self.last_center_pos[1]

        ratio = 1
        yaw = self.robot_theta
        v_yaw = math.atan2(dy, dx)
        error = v_yaw - yaw
        if error < -math.pi:
            error += 2*math.pi
        elif error > math.pi:
            error -= 2*math.pi
        error = abs(error)
        if error > math.pi/2:
            ratio = -ratio

        return ratio * math.sqrt((dx/dt)**2 + (dy/dt)**2)
    
    def calc_target_heading(self, waypoint):
        delta_dis = waypoint - self.robot_center_pos

        return np.arctan2(delta_dis[1], delta_dis[0])

    def plot(self, ax=None, re=False):
        if ax is None:
            ax = plt.gca()
        if not re:
            ax.add_patch(self.robot_patch)
        else:
            self.robot_patch.angle = np.rad2deg(self.robot_theta)
            self.robot_patch.set_xy(self.get_patch_center()/self.resolution)
        
        self.occ_manager.plot(ax, re, color='red', resolution=self.resolution)

def main():
    map_file='envs/maps/obstacle_map_occupancy_1.png'
    global resolution
    gridmap = OccupancyGridMap.from_png(map_file, resolution)
    pM = PathManager(gridmap)
    start_position = np.array([1.0, 1.0])
    end_position = np.array([4.5, 5.2])
    path = pM.find_path(start_position, end_position)
    path = pM.filter_path()
    # path = pM.find_gradient()
    ref_path, _ = pM.spline_interpolate(path)

    pid_controller = PIDController(longtitude_w=[1.0, 0.05, 0.01], latitude_w=[0.1, 0.01, 5.0])

    laser = Laser(beams=256)
    laser.set_map(gridmap)

    laser_points = None
    scat_points = None

    def update_laser_points(points, re=False):
        global laser_points

        if not re:
            laser_points = patches.PathPatch(patches.Path(points, closed=False), color='red', fill=False, linestyle=':')
        else:
            laser_points.set_path(patches.Path(points, closed=False))
        return laser_points

    def update_scatter_points(points, ax=None, re=False):
        global scat_points

        if ax is None:
            ax = plt.gca()

        if not re:
            scat_points = ax.scatter(points[:, 0], points[:, 1], s=2, color='red')
        else:
            scat_points.set_offsets(points)

    pM.plot_map()
    pM.plot_interpolated_path()
    pM.plot_path()

    robot = Robot(start_position, 90, resolution=resolution) # the angle is degree
    ax = plt.gca()
    # ax.add_patch(update_laser_points([[0, 0]])) # path form
    update_scatter_points(laser.state2obs(robot.robot_center_pos, robot.robot_theta)['point'], ax=ax)
    robot.plot(ax=ax, re=False)
    
    target_instacne = ax.scatter(start_position[0]/resolution, start_position[1]/resolution, s=1/resolution, color='green')
    achieve = False
    target_speed = 0.5

    while not achieve:
        robot_position = robot.get_transaltion()
        current_heading = robot.get_rotation()
        current_speed = robot.get_velocity()
        target_ref = get_ref_target_point([robot_position[0], robot_position[1], current_heading], ref_path)
        target_heading = robot.calc_target_heading(target_ref[0:2])
        vel, steer = pid_controller.run_step(target_speed, current_speed, target_heading, current_heading)

        print('target_heading: %.4f, current_heading: %.4f, current_speed: %.4f, input_speed: %.4f, steer: %.4f' % 
              (target_heading, current_heading, current_speed, vel+current_speed, steer))
        robot.update_integrate_state([vel+current_speed, steer], plot=True, ax=ax, re=True)

        # for display target position
        target_instacne.set_offsets(target_ref[0:2]/resolution)

        # laser points
        de_obs = laser.state2obs(robot_position, current_heading)
        # for display laser points
        update_scatter_points(de_obs['point'], ax=ax, re=True)  
        
        if (np.linalg.norm(robot_position-end_position) < 0.05):
            achieve = True

        plt.pause(0.05)

if __name__ == '__main__':
    main()