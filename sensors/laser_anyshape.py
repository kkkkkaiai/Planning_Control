'''
Reference: https://github.com/taewankim1/sequential_convex_programming
Author: taewankim1, Kai CHEN
License: MIT
'''


from sensors.sensor import Sensor
import multiprocessing as mp
from copy import copy
import numpy as np

class Laser(Sensor):
    def __init__(self, time=0, beams=64) -> None:
        super().__init__(time)
        self._dtheta     = 2*np.pi/beams# delta theta between two beams
        self._num_sensor = int(beams+1) # number of beams
        
        # corresponding angle of each beam
        self._theta_sensor = [i*self._dtheta for i in range(self._num_sensor)]
        self._N_theta = len(self._theta_sensor) # number of thetas/laser beams' angles
        self._N_laser = 100 # detection range's resolution of one line, the larger the more accurate
        self._length_laser = 2 # laser's detection range
        self._d_laser = self._length_laser / self._N_laser # laser resolution
        self._gridmap = None
        self._mapres = None

    def set_map(self, gridmap):
        self._gridmap = gridmap
        self._mapres = gridmap.cell_size
        
    def first_laser_line(self, x, yaw):
        '''
        calculate the first line of laser
        param: x: state of the robot
        return: line: the position of first line of laser
        '''
        line = np.array([x[0] + self._length_laser* np.cos(self._theta_sensor[0]+yaw), 
                         x[1] + self._length_laser* np.sin(self._theta_sensor[0]+yaw)])
        return line

    def max_detect_distance(self):
        return self._length_laser

    def sub_check(self, points):
        for idx in range(len(points)):
            if self._gridmap.is_occupied(points[idx]):
                return idx
        
        return -1

    def check_obstacle(self, points):
        obstacle_ids = np.full(points.shape[1], -1)
        for idx in range(points.shape[1]):
            points_in_beam = points[:, idx]*self._mapres
            obstacle_ids[idx] = self.sub_check(points_in_beam)
        
        return obstacle_ids

    def state2obs(self, x, yaw, filter_unknow=True):
        observ = {} # observation result, include length and point's position
        length_list = []
        point_list = []
        point = []

        point_mat = np.zeros((self._N_laser, self._N_theta, 2)) 
        theta_sensor_with_yaw = list(map(lambda x: x+yaw, self._theta_sensor))

        for idx_point in range(1, self._N_laser+1) :
            r = round(idx_point * self._d_laser, 4)
            point_mat[idx_point-1, :, 0] = x[0]/self._mapres + r/self._mapres * np.cos(theta_sensor_with_yaw)
            point_mat[idx_point-1, :, 1] = x[1]/self._mapres + r/self._mapres * np.sin(theta_sensor_with_yaw)


        obstacle_flag = self.check_obstacle(point_mat)
        obs_angle = self._d_laser * np.ones(len(theta_sensor_with_yaw))
        
        
        point = np.zeros((len(theta_sensor_with_yaw), 2))
        for idx_point in range(1, self._N_theta+1) :
            # r = round(idx_point * self.d_laser,4)
            idx = obstacle_flag[idx_point-1]
            point[idx_point-1] = point_mat[idx,idx_point-1]
            obs_angle[idx_point-1] = round(idx * self._d_laser, 4)
            length_list.append(obs_angle[idx_point-1])


        if filter_unknow:
            index = np.where(obs_angle >0)
            
            obs_angle = obs_angle[index]
            point = point[index]

        point_list.append(point)
        observ['point'] = np.array(point_list).squeeze()
        observ['length'] = np.array(length_list)

        return observ