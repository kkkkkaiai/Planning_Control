import sys, os
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from planner.trajectory_generator.spline_interpolate import *
import copy

import numpy as np
import matplotlib.pyplot as plt
import casadi as ca 
import matplotlib as mpl

V_expect = 0
class ScenarioGenerator:
    def __init__(self):
        self.generateNormalScenario()

    def generateNormalScenario(self):
        self.v0 = 0.0
        self.a0 = 0.0
        self.ds = 0.1
        self.max_vel = 1.0
        self.max_acc = 0.5
        self.max_jerk = 0.3
        self.min_acc = -0.5
        self.min_jerk = -0.3
        self.max_w = 1.0
        self.max_dw = 0.6
        self.max_ddw = 0.3
        self.min_dw = -self.max_w
        self.min_ddw = -self.max_dw

class TrajData:
    def __init__(self) -> None:
        self.velocity = []
        self.position = []
        self.time = []

    def resize(self, size):
        self.velocity = np.zeros(size)
        self.position = np.zeros((size,2))
        self.time = np.zeros(size)

class VelocityFilter:
    def __init__(self) -> None:
        self.traj_data = TrajData()

    def modify_maximum_veolicty(self, position, original_max_vels):
        self.traj_data.resize(len(position))
        self.traj_data.velocity[0] = original_max_vels[0]
        self.traj_data.position[0] = 0.0
        self.traj_data.time[0] = 0.0
        t = 0.0

        # params
        self.v0 = 0.0
        self.a0 = 0.0
        self.vg = 0.0
        self.ds = 0.0
        self.a_max = 0.0
        self.a_min = 0.0
        self.j_max = 0.0
        self.j_min = 0.0
        self.w_max = 0.0
        self.dw_max = 0.0

        for i in range(0, len(original_max_vels)-1):
            dt = np.linalg.norm(position[i+1] - position[i])/original_max_vels[i]
            t += dt
            self.traj_data.position[i] = position[i]
            self.traj_data.velocity[i] = original_max_vels[i]
            self.traj_data.time[i] = t
        
    def forward_jerk_filter(self, v0, a0, a_max, j_max, ds, original_vel, curvature=None):
        def apply_limits(vel, acc, i):
            v_lim = original_vel[i]
            ep = 1.0e-5
            if vel > v_lim + ep:
                vel = v_lim
                acc = 0.0
            
            if vel < 0.0:
                vel = v_lim
                acc = 0.0
            
            return vel, acc
        
        filtered_vels = np.zeros(len(original_vel))
        filtered_accs = np.zeros(len(original_vel))

        current_vel = v0;
        current_acc = a0;
        current_vel, current_acc = apply_limits(current_vel, current_acc, 0)

        filtered_vels[0] = current_vel
        filtered_accs[0] = current_acc

        for i in range(1, len(original_vel)-1):
            # x = x0 + v0t + 1/2*a0*t^2 + 1/6*j0*t^3
            max_v_dt = np.power(6.0*ds/j_max, 1.0/3.0) # (6*ds/j_max)**(1/3)
            global V_expect

            # calc the angle between the current point and the next point
            acc = True
            dt = 0
            desired_v = 0

            if curvature is not None:
                # desired_v = self.w_max/curvature[i-1]
                # # max_w_dt = np.power(2.0*d_theta/self.dw_max, 2.0)
                # if desired_v < current_vel:
                #     acc = False

                dt = min(ds/max(current_vel, 1e-6), max_v_dt, ds/max(V_expect[i-1], 1e-6)) # velocity
                # print(ds/max(current_vel, 1e-6), ds/desired_v, max_v_dt)
            else:
                dt = min(ds/max(current_vel, 1e-6), max_v_dt, ds/max(V_expect[i-1], 1e-6)) # velocity
            
            print(ds/max(current_vel, 1e-6), max_v_dt, ds/max(V_expect[i-1], 1e-6))
            if acc is True:
                if current_acc + j_max*dt >= a_max:
                    # if the acceleration is larger than the maximum acceleration, 
                    # then the acceleration is limited by the maximum acceleration
                    tmp_jerk = min((a_max - current_acc)/dt, j_max)
                    current_vel = current_vel + current_acc*dt + 0.5*tmp_jerk*dt*dt
                    current_acc = a_max
                else:
                    current_vel = current_vel + current_acc*dt + 0.5*j_max*dt*dt
                    current_acc = current_acc + j_max*dt
            else:
                # if the acceleration is smaller than the minimum acceleration,
                if current_acc + self.j_min*dt <= self.a_min:
                    tmp_jerk = max((self.a_min - current_acc)/dt, self.j_min)
                    current_vel = current_vel + current_acc*dt + 0.5*tmp_jerk*dt*dt
                    current_acc = self.a_min
                else:
                    current_vel = current_vel + current_acc*dt + 0.5*self.a_min*dt*dt
                    current_acc = current_acc + self.a_min*dt  

            current_vel, current_acc = apply_limits(current_vel, current_acc, i)

            filtered_vels[i] = current_vel
            filtered_accs[i] = current_acc

        return filtered_vels, filtered_accs

    def backward_jerk_filter(self, v0, a0, a_min, j_min, ds, original_vel, filtered_vels, filtered_accs):
        input_rev = copy.copy(original_vel)

        input_rev = np.flip(input_rev)
        filtered_vels, filtered_accs = self.forward_jerk_filter(v0, np.abs(a0), np.abs(a_min), np.abs(j_min), ds, input_rev)
        filtered_vels = np.flip(filtered_vels)
        filtered_accs = np.flip(filtered_accs)

        for i in range(len(filtered_accs)):
            filtered_accs[i] = -filtered_accs[i]

        return filtered_vels, filtered_accs

    def merge_filtered_velocity(self, forward_vels, backward_vels):
        v0 = forward_vels[0]

        merged_vels = np.zeros(len(forward_vels))
        
        i = 0
        if backward_vels[0] < v0 - 1e-6:
            while backward_vels[i] < forward_vels[i] and i < len(merged_vels):
                merged_vels[i] = forward_vels[i]
                i += 1

        while i < len(merged_vels):
            merged_vels[i] = forward_vels[i] if forward_vels[i] < backward_vels[i] else backward_vels[i]
            i += 1

        return merged_vels

    def smooth_velocity(self, ds, v0, a0, a_max, a_min, j_max, j_min, w_max, dw_max, original_vel, curvature):
        self.ds = ds
        self.v0 = v0
        self.a0 = a0
        self.a_max = a_max
        self.a_min = a_min
        self.j_max = j_max
        self.j_min = j_min
        self.w_max = w_max
        self.dw_max = dw_max

        forward_vels, forward_accs = self.forward_jerk_filter(v0, a0, a_max, j_max, ds, original_vel, curvature)
        backward_vels, backward_accs = self.backward_jerk_filter(v0, a0, a_min, j_min, ds, original_vel, forward_vels, forward_accs)
        merged_velocity = self.merge_filtered_velocity(forward_vels, backward_vels)

        return merged_velocity, forward_accs, backward_accs


# generate 5 random points
generate_num = 10
end_point = 7
x = np.linspace(0, end_point, generate_num)
y = np.random.rand(generate_num)
points = np.array([x, y])

# generate spline interpolation
si = Spline2D(x, y)
interpolate_points = []
ratio = 20

interpolate_points_index = np.arange(0, si.s[-1], 1/ratio)

for i_s in interpolate_points_index:
    interpolate_points.append(si.calc_position(i_s))

scene_gen = ScenarioGenerator()
scene_gen.ds = 1/ratio
max_vels = np.array([scene_gen.max_vel]*len(interpolate_points_index))

# https://blog.csdn.net/q1302182594/article/details/50556529
dddy = []
dx = []
dy = []

for i in range(0, len(interpolate_points_index)-1):
    dy.append(interpolate_points[i+1][1]-interpolate_points[i][1])
    dx.append(interpolate_points[i+1][0]-interpolate_points[i][0])
    dddy.append(dy[-1]/dx[-1])

K = [] # curvature(start from the second point)
ddx = []
ddy = []
for i in range(0, len(interpolate_points_index)-2):
    ddx.append(dx[i+1]-dx[i])
    ddy.append(dy[i+1]-dy[i])
    K.append(abs(dx[i]*ddy[i]-dy[i]*ddx[i])/(dx[i]**2+dy[i]**2)**(3/2))

V_expect = scene_gen.max_w / np.array(K)
# let the values in V_expect are not larger than scene_gen.max_v
for i in range(len(V_expect)):
    if V_expect[i] > scene_gen.max_vel:
        V_expect[i] = scene_gen.max_vel

# filter the velocity
velocity_filter = VelocityFilter()
velocity_filter.modify_maximum_veolicty(np.array(interpolate_points), max_vels)

merged_velocity, forward_acc, backward_acc = velocity_filter.smooth_velocity(scene_gen.ds, scene_gen.v0, scene_gen.a0, scene_gen.max_acc, scene_gen.min_acc, scene_gen.max_jerk, scene_gen.min_jerk, 
                                                  scene_gen.max_w, scene_gen.max_dw,  max_vels, K)

cmap = plt.cm.get_cmap('jet')
normalize = plt.Normalize(vmin=min(merged_velocity), vmax=max(merged_velocity))


plt.scatter(interpolate_points_index, np.array(interpolate_points)[:, 1],  c=cmap(normalize(merged_velocity)))
# display a colorbar with the given velocity data


plt.colorbar(mpl.cm.ScalarMappable(norm=normalize, cmap=cmap))
# plt.plot(points[0, :], points[1, :], 'ro')
plt.axis('equal')

# plot the velocity in a new figure
plt.figure()
plt.plot(interpolate_points_index, merged_velocity, label='velocity')
plt.plot(interpolate_points_index, forward_acc, label='f_acc')

# plot the expected velocity
plt.plot(interpolate_points_index[1:-1], V_expect, label='expected velocity')

# plot the curvature on the velocity figure
plt.plot(interpolate_points_index[1:-1], K, label='curvature')
# plt.plot(interpolate_points_index, point_direction, label='direction')

# plot the w/v on the velocity figure
w_v = [scene_gen.max_w / scene_gen.max_vel] * len(interpolate_points_index)
plt.plot(interpolate_points_index, w_v, label='w/v')


# plot accelaration in every step
# accelaration = np.diff(merged_velocity)/scene_gen.ds
# print(accelaration)
# plt.plot(interpolate_points_index[:-1], accelaration, label='accelaration')
# # plot max acc and min acc
# plt.plot(interpolate_points_index, [scene_gen.max_acc]*len(interpolate_points_index), label='max acc')
# plt.plot(interpolate_points_index, [scene_gen.min_acc]*len(interpolate_points_index), label='min acc')
# # plot jerk in every step
# jerk = np.diff(accelaration)/scene_gen.ds
# plt.plot(interpolate_points_index[:-2], jerk, label='jerk')
# # plot max jerk and min jerk
# plt.plot(interpolate_points_index, [scene_gen.max_jerk]*len(interpolate_points_index), label='max jerk')
# plt.plot(interpolate_points_index, [scene_gen.min_jerk]*len(interpolate_points_index), label='min jerk')

plt.axis('equal')

plt.figure()
# plot points with expected velocity
plt.scatter(interpolate_points_index[1:-1], np.array(interpolate_points)[:, 1][1:-1],  c=cmap(normalize(V_expect)))

# plot the points
plt.legend()
plt.show()

