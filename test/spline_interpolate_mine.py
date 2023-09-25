import sys, os
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from planner.trajectory_generator.spline_interpolate import *
import copy

import time
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca 
import matplotlib as mpl

import casadi as ca
import osqp
from scipy import sparse

V_expect = 0
class ScenarioGenerator:
    def __init__(self):
        self.generateNormalScenario()

    def generateNormalScenario(self):
        self.v0 = 0.0
        self.a0 = 0.0
        self.vg = 0.0
        self.ag = 0.0
        self.ds = 0.1
        self.max_vel = 1.0
        self.max_acc = 0.8
        self.max_jerk = 0.4
        self.min_acc = -0.8
        self.min_jerk = -0.4
        self.max_w = 1.0
        self.max_dw = 0.5
        self.max_ddw = 0.4
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

        for i in range(0, len(original_max_vels)):
            self.traj_data.position[i] = position[i]
            self.traj_data.velocity[i] = original_max_vels[i]
  
        
    def forward_jerk_filter(self, v0, a0, a_max, j_max, ds, original_vel):
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

            # calc the angle between the current point and the next point
            dt = min(ds/max(current_vel, 1e-6), max_v_dt) # velocity

            if current_acc + j_max*dt >= a_max:
                # if the acceleration is larger than the maximum acceleration, 
                # then the acceleration is limited by the maximum acceleration
                tmp_jerk = min((a_max - current_acc)/dt, j_max)
                current_vel = current_vel + current_acc*dt + 0.5*tmp_jerk*dt*dt
                current_acc = a_max
            else:
                current_vel = current_vel + current_acc*dt + 0.5*j_max*dt*dt
                current_acc = current_acc + j_max*dt
            # else:
            #     # if the acceleration is smaller than the minimum acceleration,
            #     if current_acc + self.j_min*dt <= self.a_min:
            #         tmp_jerk = max((self.a_min - current_acc)/dt, self.j_min)
            #         current_vel = current_vel + current_acc*dt + 0.5*tmp_jerk*dt*dt
            #         current_acc = self.a_min
            #     else:
            #         current_vel = current_vel + current_acc*dt + 0.5*self.a_min*dt*dt
            #         current_acc = current_acc + self.a_min*dt  

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

    def smooth_velocity(self, ds, v0, a0, a_max, a_min, j_max, j_min, w_max, dw_max, original_vel):
        self.ds = ds
        self.v0 = v0
        self.a0 = a0
        self.a_max = a_max
        self.a_min = a_min
        self.j_max = j_max
        self.j_min = j_min
        self.w_max = w_max
        self.dw_max = dw_max

        forward_vels, forward_accs = self.forward_jerk_filter(v0, a0, a_max, j_max, ds, original_vel)
        backward_vels, backward_accs = self.backward_jerk_filter(v0, a0, a_min, j_min, ds, original_vel, forward_vels, forward_accs)
        merged_velocity = self.merge_filtered_velocity(forward_vels, backward_vels)
        optimized_velocity = self.optimize_velocity_SX(merged_velocity)
        
        self.calculate_time(merged_velocity)

        return merged_velocity, optimized_velocity, forward_accs, backward_accs, 

    def optimize_velocity(self, merged_velocity):
        # the objective function is velocity
        # the constraints are acceleration, jerk
        
        opti = ca.Opti()
        opt_vel = opti.variable(len(merged_velocity))
        opt_acc = opti.variable(len(merged_velocity))

        obj = 0
        for i in range(len(merged_velocity)):
            obj += -opt_vel[i]

        opti.minimize(obj)
        opti.subject_to(opti.bounded(self.a_min, opt_acc, self.a_max))
        for i in range(len(merged_velocity)):
            # v_i+1^2 - v_i^2 = 2*ai(si+1 - si)
            opti.subject_to(opt_vel[i] >= 0)
            opti.subject_to(opt_vel[i] <= merged_velocity[i])
            opti.subject_to(opt_acc[i] >= self.a_min)
            opti.subject_to(opt_acc[i] <= self.a_max)
            if i < len(merged_velocity)-1:
                opti.subject_to(opt_vel[i+1]**2 - opt_vel[i]**2 == 2*opt_acc[i]*self.ds)
                opti.subject_to((opt_acc[i+1] - opt_acc[i]) <= self.j_max*self.ds/merged_velocity[i])
                opti.subject_to((opt_acc[i+1] - opt_acc[i]) >= self.j_min*self.ds/merged_velocity[i])

        opts_setting = {'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time': 0, 'expand': True,
                        'ipopt.acceptable_tol': 1e-6, 'ipopt.acceptable_obj_change_tol': 1e-5}

        opti.solver('ipopt', opts_setting)
        start_time = time.time()
        sol = opti.solve()
        opt_vel = sol.value(opt_vel)
        end_time = time.time()
        print('optimize_time_ca ', end_time - start_time)
        print(opt_vel.shape)
        return opt_vel

    def optimize_velocity_SX(self, merged_velocity):
        # use lp to solve
        
        opt_vel = ca.SX.sym('opt_vel', len(merged_velocity))
        opt_acc = ca.SX.sym('opt_acc', len(merged_velocity))

        obj = 0
        for i in range(len(merged_velocity)):
            obj += -opt_vel[i]

        lbx = [] # the constraint of x
        ubx = []
        lbg = [] # the constraint of g
        ubg = []

        for i in range(len(merged_velocity)):
            lbx.append(0)
            ubx.append(merged_velocity[i])
        for i in range(len(merged_velocity)):
            lbx.append(self.a_min)
            ubx.append(self.a_max)

        g = []
        for i in range(len(merged_velocity)-1):
                g.append(opt_vel[i+1]**2 - opt_vel[i]**2 - 2*opt_acc[i]*self.ds)
                lbg.append(0)
                ubg.append(0)
                g.append((opt_acc[i+1] - opt_acc[i])/self.ds)
                lbg.append(self.j_min)
                ubg.append(self.j_max)

        lp = {'x': ca.vertcat(opt_vel, opt_acc), 'f': obj, 'g': ca.vertcat(*g)}
        # use osqp and set print level to 0
        # set max iteration to 100
        # solver = ca.qpsol('solver', 'qpoases', lp, {'sparse': True, 'expand': True}) 
        solver = ca.nlpsol('solver', 'ipopt', lp, {'ipopt': {'print_level': 0}, 'print_time': 0, 'expand': True})

        start_time = time.time()
        sol = solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        opt_vel = np.array(sol['x']).T[0, :len(merged_velocity)]
        end_time = time.time()
        print('optimize_time_ca ', end_time - start_time)

        return np.array(opt_vel)

    def calculate_time(self, merged_velocity):
        t = 0.0
        for i in range(1,len(self.traj_data.position)):
            self.traj_data.velocity[i] = merged_velocity[i]
            
            dt = np.linalg.norm(self.traj_data.position[i] - self.traj_data.position[i-1])/((merged_velocity[i]+merged_velocity[i-1])/2)
            # print(np.linalg.norm(self.traj_data.position[i] - self.traj_data.position[i-1]), "  ", ((merged_velocity[i]+merged_velocity[i-1])/2), "    " ,dt)
            t += dt
            # print(t)
            self.traj_data.time[i] = t
        


# generate 5 random points
generate_num = 10
end_point = 7
x = np.linspace(0, end_point, generate_num)
y = np.random.rand(generate_num)
points = np.array([x, y])

# generate spline interpolation
si = Spline2D(x, y)
interpolate_points = []
ratio = 10

interpolate_points_index = np.arange(0, si.s[-1], 1/ratio)

for i_s in interpolate_points_index:
    interpolate_points.append(si.calc_position(i_s))

scene_gen = ScenarioGenerator()
scene_gen.ds = 1/ratio

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
V_expect = np.insert(V_expect, 0, scene_gen.v0)
V_expect = np.append(V_expect, scene_gen.vg)

# filter the velocity
velocity_filter = VelocityFilter()
velocity_filter.modify_maximum_veolicty(np.array(interpolate_points), V_expect)

merged_velocity, optimized_velocity, forward_acc, backward_acc = velocity_filter.smooth_velocity(scene_gen.ds, scene_gen.v0, scene_gen.a0, 
                                                                 scene_gen.max_acc, scene_gen.min_acc, scene_gen.max_jerk, scene_gen.min_jerk, 
                                                                 scene_gen.max_w, scene_gen.max_dw,  V_expect)

cmap = plt.cm.get_cmap('jet')
normalize = plt.Normalize(vmin=min(merged_velocity), vmax=max(merged_velocity))


plt.scatter(np.array(interpolate_points)[:, 0], np.array(interpolate_points)[:, 1], c=cmap(normalize(merged_velocity)))
# display a colorbar with the given velocity data

plt.colorbar(mpl.cm.ScalarMappable(norm=normalize, cmap=cmap))
# plt.plot(points[0, :], points[1, :], 'ro')
plt.axis('equal')

# plot the velocity in a new figure
plt.figure()

plt.plot(np.array(interpolate_points)[:, 0], forward_acc, label='f_acc')

# plot the expected velocity
# plt.plot(np.array(interpolate_points)[:, 0], V_expect, label='expected velocity')
plt.plot(np.array(interpolate_points)[:, 0], merged_velocity, label='merged_velocity')
plt.plot(np.array(interpolate_points)[:, 0], optimized_velocity, label='optimized velocity')

# plot the curvature on the velocity figure
plt.plot(np.array(interpolate_points)[:, 0][1:-1], K, label='curvature')
# plt.plot(interpolate_points_index, point_direction, label='direction')


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


# plot the points
plt.legend()
plt.show()

