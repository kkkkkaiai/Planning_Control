"""
author: Kai CHEN
mail: chenkai0130@outlook.com
license: MIT
"""
from abc import ABCMeta

import polytope as pt
import matplotlib.patches as patches
import numpy as np
import matplotlib.pyplot as plt
import casadi as ca


def exp_func(x, epsilon):
    return np.sign(x) * (np.abs(x) ** epsilon)


def rotate_func(phi=0, is_radian=False):
    if not is_radian:
        phi = np.deg2rad(phi)
    matrix = np.array([[np.cos(phi), -np.sin(phi)],
                      [np.sin(phi), np.cos(phi)]])
    return matrix


class ConvexRegion2D:
    __metaclass__ = ABCMeta

    def get_convex_rep(self):
        raise NotImplementedError()

    def get_plot_patch(self):
        raise NotImplementedError


class RectangleRegion(ConvexRegion2D):
    def __init__(self, left, right, down, up):
        self.left = left
        self.right = right
        self.down = down
        self.up = up

    def get_convex_rep(self):
        mat_A = np.array([[-1, 0], [0, -1], [1, 0], [0, 1]])
        vec_b = np.array([[-self.left], [-self.down], [self.right], [self.up]])
        return mat_A, vec_b

    def get_plot_patch(self, ec="k", fc="r"):
        return patches.Rectangle(
            (self.left, self.down),
            self.right - self.left,
            self.up - self.down,
            linewidth=1,
            edgecolor=ec,
            facecolor=fc,
        )


class PolytopeRegion(ConvexRegion2D):
    def __init__(self, mat_A, vec_b):
        self.mat_A = mat_A
        self.vec_b = vec_b
        self.points = pt.extreme(pt.Polytope(mat_A, vec_b))

    @classmethod
    def convex_hull(self, points):
        """Convex hull of N points in d dimensions as Nxd numpy array"""
        P = pt.reduce(pt.qhull(points))
        return PolytopeRegion(P.A, P.b)

    def get_convex_rep(self):
        return self.mat_A, self.vec_b.reshape(self.vec_b.shape[0], -1)

    def get_plot_patch(self, ec="k", fc="r"):
        return patches.Polygon(self.points, closed=True, linewidth=1, edgecolor=ec, facecolor=fc)


def get_dist_point_to_region(point, mat_A, vec_b):
    """Return distance between a point and a convex region"""
    opti = ca.Opti()
    # variables and cost
    point_in_region = opti.variable(mat_A.shape[-1], 1)
    cost = 0
    # constraints
    constraint = ca.mtimes(mat_A, point_in_region) <= vec_b
    opti.subject_to(constraint)
    dist_vec = point - point_in_region
    cost += ca.mtimes(dist_vec.T, dist_vec)
    # solve optimization
    opti.minimize(cost)
    option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", option)
    opt_sol = opti.solve()
    # minimum distance & dual variables
    dist = opt_sol.value(ca.norm_2(dist_vec))
    # if dist > 0:
    #     lamb = opt_sol.value(opti.dual(constraint)) / (2 * dist)
    # else:
    #     lamb = np.zeros(shape=(mat_A.shape[0],))
    return dist


def get_dist_region_to_region(mat_A1, vec_b1, mat_A2, vec_b2):
    opti = ca.Opti()
    # variables and cost
    point1 = opti.variable(mat_A1.shape[-1], 1)
    point2 = opti.variable(mat_A2.shape[-1], 1)
    cost = 0
    # constraints
    constraint1 = ca.mtimes(mat_A1, point1) <= vec_b1
    constraint2 = ca.mtimes(mat_A2, point2) <= vec_b2
    opti.subject_to(constraint1)
    opti.subject_to(constraint2)
    dist_vec = point1 - point2
    cost += ca.mtimes(dist_vec.T, dist_vec)
    # solve optimization
    opti.minimize(cost)
    option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
    opti.solver("ipopt", option)
    opt_sol = opti.solve()
    # minimum distance & dual variables
    dist = opt_sol.value(ca.norm_2(dist_vec))
    if dist > 0:
        lamb = opt_sol.value(opti.dual(constraint1)) / (2 * dist)
        mu = opt_sol.value(opti.dual(constraint2)) / (2 * dist)
    else:
        lamb = np.zeros(shape=(mat_A1.shape[0],))
        mu = np.zeros(shape=(mat_A2.shape[0],))
    return dist, lamb, mu


class SuperEllipse():
    def __init__(self, a=None, eps=None, tc=None, phi=None, num=20):
        # Shape
        if a is None:
            a = [1.0, 1.0]
        if eps is None:
            eps = 1.0

        # Pose
        if phi is None:
            phi = 0.0
        if tc is None:
            tc = [-1.0, 1.0]

        self.num = num
        self.a = a
        self.eps = eps
        self.phi = phi
        self.tc = tc
        self.plot_theta = np.linspace(0, np.pi*2, 100)
        self.plot_instance = None

    def get_points(self, is_radian=True):
        # the function is
        xy = np.array([self.a[0]*exp_func(np.cos(self.plot_theta), self.eps),
                      self.a[1]*exp_func(np.sin(self.plot_theta), self.eps)])
        xy = rotate_func(self.phi, is_radian).dot(xy)
        xy_new = xy.T + np.array(self.tc).T

        return np.array(xy_new.T)

    def update_state(self, pos, phi):
        self.tc = pos
        self.phi = phi

    def plot(self, ax=None, re=False, color=None, resolution=1.0):
        points = self.get_points()
        points = points.T/resolution

        if ax is None:
            ax = plt.axes()

        if not re:
            self.plot_instance = patches.PathPatch(patches.Path(points), color=color, fill=False)
            ax.add_patch(self.plot_instance)
        else:
            self.plot_instance.set_path(patches.Path(points))


if __name__ == '__main__':
    s = SuperEllipse([2.5, 1.8], 0.5, None, 20, 20)

    ax = plt.gca()
    ax.set_xlim(-1, 6)
    ax.set_ylim(-1, 6)
    ax.set_aspect('equal')
    s.plot(ax, color='red')
    plt.pause(1.0)
    s.update_state([4.0, 2.0], 20)
    s.plot(ax, re=True)
    plt.pause(1.0)

    

