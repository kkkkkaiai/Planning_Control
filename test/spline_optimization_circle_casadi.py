import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from planner.trajectory_generator.spline_interpolate import Spline2D
import osqp
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
import casadi as ca

def calc_2d_spline_interpolation(x, y, num=100):
    """
    Calc 2d spline course with interpolation

    :param x: interpolated x positions
    :param y: interpolated y positions
    :param num: number of path points
    :return:
        - x     : x positions
        - y     : y positions
        - yaw   : yaw angle list
        - k     : curvature list
        - s     : Path length from start point
    """
    sp = Spline2D(x, y)
    s = np.linspace(0, sp.s[-1], num+1)[:-1]

    r_x, r_y, r_yaw, r_k = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        r_x.append(ix)
        r_y.append(iy)
        r_yaw.append(sp.calc_yaw(i_s))
        r_k.append(sp.calc_curvature(i_s))

    travel = np.cumsum([np.hypot(dx, dy) for dx, dy in zip(np.diff(r_x), np.diff(r_y))]).tolist()
    travel = np.concatenate([[0.0], travel])

    return r_x, r_y, r_yaw, r_k, travel

def spline_opti():
    start_point = 0
    end_point = 10
    number = 50
    # generate the reference path
    x = np.linspace(start_point, end_point, number)
    y = np.repeat(0.05, len(x))
    points = np.array([x, y])

    # generate two obstacles(obstacle is a circle)
    obs_center = np.array([[2, 0], [5, 0], [8, 0]])
    obs_radius = np.array([0.5, 0.7, 0.9])
    obs_radius = np.array([0.9, 0.7, 0.5])

    # optimize the path with osqp
    opti = ca.Opti()

    # The problem consist several objective terms, including smooth, obstacle and guidance
    
    # decision variables
    x_opt = opti.variable(2, len(x))
    
    # weight matrix

    # smooth term with the weight matrix
    smooth_term = 0
    for i in range(len(x)-2):
        smooth_term += (x_opt[:, i+2] - 2*x_opt[:, i+1] + x_opt[:, i]) * (x_opt[:, i+2] - 2*x_opt[:, i+1] + x_opt[:, i])

    # obstacle term
    obs_term = 0
    for i in range(len(x)):
        for j in range(len(obs_center)):
            dist = ca.mtimes([(x_opt[:, i] - obs_center[j]).T, (x_opt[:, i] - obs_center[j])])
            obs_term += ca.exp(-(dist-3*obs_radius[j]))
    
    # guidance term(penalize the distance between the path and the reference path)
    # calculate the distance between the path and the reference path
    guidance_term = 0
    for i in range(len(x)):
        dist = ca.mtimes([(x_opt[:, i] - points[:, i]).T, (x_opt[:, i] - points[:, i])])
        guidance_term += dist

    # velocity term(penalize the velocity change)
    velocity_term = 0
    for i in range(len(x)-1):
        velocity_term += (x_opt[:, i+1] - x_opt[:, i]) * (x_opt[:, i+1] - x_opt[:, i])

    # objective function(the term must be scalar)
    opti.minimize(ca.sum1(smooth_term) + 1*ca.sum1(guidance_term) + 0.5*ca.sum1(obs_term) + 2*ca.sum1(velocity_term))

    # constraints
    # opti.subject_to(x_opt[:, 0] == np.array([0, 0]))
    # opti.subject_to(x_opt[:, -1] == np.array([10, 0]))

    # add random bias on the initial guess of y
    randon_init = np.random.rand(2, len(x))
    opti.set_initial(x_opt, points) # also can use the random initial guess

    # solve the problem
    opti.solver('ipopt', {'expand': False, 'ipopt.max_iter': 100, 'ipopt.print_level': 0, 'print_time':1, 'verbose': False})
    sol = opti.solve()

    # get the optimized points
    optimized_points = sol.value(x_opt)

    # plot the optimized points
    plt.scatter(optimized_points[0, :], optimized_points[1, :], s=10, color='red', label='optimized path point')

    # plot the optimized path
    sp_x, sp_y, sp_yaw, sp_k, sp_travel = calc_2d_spline_interpolation(optimized_points[0, :], optimized_points[1, :], number*2)
    plt.plot(sp_x, sp_y, color='red', label='optimized path line')

    # plot the obstacles with their radius
    for i in range(len(obs_center)):
        circle = plt.Circle(obs_center[i], obs_radius[i], color='black', fill=False)
        plt.gcf().gca().add_artist(circle)

    # plot path
    # plt.plot(points[0, :], points[1, :], label='reference path line')
    plt.scatter(x, y, s=10, color='green', label='reference path point')
    plt.plot(x, y, color='green', label='reference path line')
    plt.axis('equal')
    plt.legend()
    plt.show()



spline_opti()