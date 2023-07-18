'''
Reference: https://github.com/taewankim1/sequential_convex_programming
Author: taewankim1, Kai CHEN
License: MIT
'''


from sensors.sensor import Sensor
from copy import copy
import numpy as np

class Laser(Sensor):
    def __init__(self, ix, iu, time=0, beams=64) -> None:
        super().__init__(ix, iu, time)
        self._dtheta = 2*np.pi/beams# delta theta between two beams
        self._num_sensor = int(beams+1) # number of beams
        # corresponding angle of each beam
        self._theta_sensor = [i*self._dtheta for i in range(self._num_sensor)]
        self._N_theta = len(self._theta_sensor) # number of thetas/laser beams' angles
        self._N_laser = 300 # detection range's resolution, larger is more accurate
        self._length_laser = 6 # laser's detection range
        self._d_laser = self._length_laser / self._N_laser

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

    def check_obstacle_new(self, p_mat, c, H):
        N_data, _ = np.shape(p_mat)
        p_mat = np.expand_dims(p_mat,2)
        flag_obstacle = np.zeros(N_data) # if obstacle, True
        for c1, H1 in zip(c, H) :
            H_mat = np.repeat(np.expand_dims(H1, 0), N_data, 0)
            c_mat = np.expand_dims(np.repeat(np.expand_dims(c1, 0), N_data, 0), 2)
            flag_obstacle_e = 1 - np.linalg.norm(np.squeeze(H_mat@(p_mat-c_mat)), axis=1) >=0 
            flag_obstacle = np.logical_or(flag_obstacle, flag_obstacle_e)
        return flag_obstacle

    def state2obs(self, x, yaw, c, H, filter_unknow=True):
        observ = {} # observation result, include length and point's position
        length_list = []
        point_list = []
        obs = []
        point = []

        point_mat = np.zeros((self._N_laser, self._N_theta, 3))
        theta_sensor_with_yaw = list(map(lambda x: x+yaw, self._theta_sensor))
        for idx_point in range(1, self._N_laser+1) :
            r = round(idx_point * self._d_laser, 4)
            point_mat[idx_point-1, :, 0] = x[0] + r * np.cos(theta_sensor_with_yaw)
            point_mat[idx_point-1, :, 1] = x[1] + r * np.sin(theta_sensor_with_yaw)

        v, h, d = np.shape(point_mat)
        point_mat = np.reshape(point_mat, (v*h,d))
        flag_obstacle = self.check_obstacle_new(point_mat, c, H)
        point_mat = np.reshape(point_mat, (v,h,d))
        flag_obstacle = np.reshape(flag_obstacle, (v,h,1))

        obs = self._d_laser * np.ones(len(theta_sensor_with_yaw))
        point = np.zeros((len(theta_sensor_with_yaw), 2))
        flag_not_meet_obstacle = np.ones(len(theta_sensor_with_yaw))
        for idx_point in range(1, self._N_laser+1) :
            # r = round(idx_point * self.d_laser,4)
            point[:,0][flag_not_meet_obstacle==True] = point_mat[idx_point-1,:,0][flag_not_meet_obstacle==True]
            point[:,1][flag_not_meet_obstacle==True] = point_mat[idx_point-1,:,1][flag_not_meet_obstacle==True]

            flag_not_obs = np.logical_not(flag_obstacle[idx_point-1,:,0])
            flag_not_meet_obstacle = np.logical_and(flag_not_meet_obstacle,flag_not_obs)
            obs += flag_not_meet_obstacle * self._d_laser

        if filter_unknow:
            index = np.where(obs <= self._length_laser)
            obs = obs[index]
            point = point[index]
            sensor_theta = np.array(self._theta_sensor)[index]
        else:
            obs[obs > self._length_laser] = self._length_laser
            sensor_theta = copy(np.array(self._theta_sensor))

        # print(sensor_theta)
        length_list.append(obs)
        point_list.append(point)
        observ['length'] = np.array(length_list).squeeze()
        observ['point'] = np.array(point_list).squeeze()
        observ['theta'] = sensor_theta

        return observ