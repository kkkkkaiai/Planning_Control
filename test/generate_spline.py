import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from planner.trajectory_generator.spline_interpolate import *
import numpy as np
import matplotlib.pyplot as plt

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

def test1d():
    x = [-0.5, 0.0, 0.5, 1.0, 1.5]
    y = [3.2, 2.7, 6, 5, 6.5]
    sp = Spline(x, y)

    x_list = np.linspace(x[0], x[-1], 1000, endpoint=False)
    y_list = []

    for i in x_list:
        y_list.append(sp.calc(i))

    #plot raw points
    plt.scatter(x, y, color='red', marker='o', label='raw points')
    plt.plot(x_list, y_list)
    plt.show()

def test2d():
    input_x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    input_y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]

    x, y, yaw, k, travel = calc_2d_spline_interpolation(input_x, input_y, num=200)

    print("travel: ", travel)

    plt.subplots(1)
    plt.plot(input_x, input_y, "xb", label="input")
    plt.plot(x, y, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(travel, [math.degrees(i_yaw) for i_yaw in yaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(travel, k, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()

# test1d()
test2d()

