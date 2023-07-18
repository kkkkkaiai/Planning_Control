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

def exp_func(x, epsilon):
    return np.sign(x) * (np.abs(x) ** epsilon)

def rotate_func(phi=0, is_radian=False):
    if not is_radian:
        phi = np.deg2rad(phi)
    matrix = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
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

    def get_plot_patch(self):
        return patches.Rectangle(
            (self.left, self.down),
            self.right - self.left,
            self.up - self.down,
            linewidth=1,
            edgecolor="k",
            facecolor="r",
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

    def get_plot_patch(self):
        return patches.Polygon(self.points, closed=True, linewidth=1, edgecolor="k", facecolor="r")

class PolytopeDecomposition(ConvexRegion2D):
    def __init__(self):
        pass    

    def get_convex_rep(self):
        pass

    def get_plot_patch(self):
        pass
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
            tc = [0.0, 0.0]

        self.num = num
        self.a = a
        self.eps = eps
        self.phi = phi
        self.tc = tc

    def get_points(self):
        theta = np.linspace(0, np.pi*2, 100)
        xy = np.array([self.a[0]*exp_func(np.cos(theta), self.eps), self.a[1]*exp_func(np.sin(theta), self.eps)])
        
        xy = xy.T + np.array(self.tc)
        xy_new = rotate_func(self.phi).dot(xy.T)

        return xy_new

    def plot(self, ax=None):
        points = self.get_points()

        if ax is None:
            ax = plt.axes()
        
        ax.set_aspect('equal')
        ax.plot(points[0], points[1])
        
        return points

if __name__ == '__main__':
    s = SuperEllipse([2.5, 1.8], 0.5, None, 45, 20)
    s.plot()
    plt.show()