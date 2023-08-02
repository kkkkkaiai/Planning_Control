import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from models.python.geometry_utils import SuperEllipse
import numpy as np
import matplotlib.pyplot as plt

# generate two obstacles(obstacle is a superellipse)
# test1
obs_center = np.array([[4, 0], [5, 0], [7, 0]])
obs_param = np.array([[0.6, 0.4], [0.4, 0.8], [0.2, 0.15]])
sample_x = np.linspace(3, 7.5, 1000)

# test2
# obs_center = np.array([[0.0, 0]])
# obs_param = np.array([[0.4, 0.3]])
# sample_x = np.linspace(-1, 1, 20)

sample_y = np.sin(sample_x)/3
sample_point = np.array([sample_x, sample_y]).T

obs_instance = []
for i in range(len(obs_center)):
    obs = SuperEllipse(obs_param[i], 4, obs_center[i], 3.0, 20)
    obs_instance.append(obs)

# plot the superellipse
fig, ax = plt.subplots()
for i in range(len(obs_center)):
    obs_instance[i].plot(ax)

def calc_distance_to_superellipse_surface(x, y, obs_instance):
        """
        calculate the distance between the point and the superellipse
        """
        dists = []
        points = []
        for i in range(len(obs_instance)):
            dist, xy = obs_instance[i].calc_distance(x, y)
            dists.append(dist)
            points.append(xy)
        
        return dists, points

first = True
raw_points = point1 = point2 = point3 = None
plot1 = plot2 = plot3 = None
plot4 = plot5 = plot6 = None
text1 = text2 = text3 = None

for i in range(len(sample_point)):
    dists, points = calc_distance_to_superellipse_surface(sample_point[i][0], sample_point[i][1], obs_instance)

    if first:
        first = False
        raw_points = plt.scatter(sample_point[i][0], sample_point[i][1], s=10, color='green', label='sample point')
        point1 = plt.scatter(points[0][0], points[0][1], s=10, color='green', label='closest point')
        text1 = plt.text(points[0][0], points[0][1], 'dist: %.3f' % dists[0], fontsize=10)
        plot1 = plt.plot([sample_point[i][0], points[0][0]], [sample_point[i][1], points[0][1]], color='red', label='distance')
        point2 = plt.scatter(points[1][0], points[1][1], s=10, color='green')
        text2 = plt.text(points[1][0], points[1][1], 'dist: %.3f' % dists[1], fontsize=10)
        plot2 = plt.plot([sample_point[i][0], points[1][0]], [sample_point[i][1], points[1][1]], color='red')
        point3 = plt.scatter(points[2][0], points[2][1], s=10, color='green')
        text3 = plt.text(points[2][0], points[2][1], 'dist: %.3f' % dists[2], fontsize=10)
        plot3 = plt.plot([sample_point[i][0], points[2][0]], [sample_point[i][1], points[2][1]], color='red')
        # plot the point to the obstacle's center
        plot5 = plt.plot([sample_point[i][0], obs_center[0][0]], [sample_point[i][1], obs_center[0][1]], color='blue', label='distance to center')
        plot6 = plt.plot([sample_point[i][0], obs_center[1][0]], [sample_point[i][1], obs_center[1][1]], color='blue')
        plot7 = plt.plot([sample_point[i][0], obs_center[2][0]], [sample_point[i][1], obs_center[2][1]], color='blue')
        plt.axis('equal')
        plt.legend()
    else:
        raw_points.set_offsets(np.c_[sample_point[i][0], sample_point[i][1]])
        point1.set_offsets(np.c_[points[0][0], points[0][1]])
        text1.set_position((points[0][0], points[0][1]))
        text1.set_text('dist: %.3f' % dists[0])
        plot1[0].set_data([sample_point[i][0], points[0][0]], [sample_point[i][1], points[0][1]])
        point2.set_offsets(np.c_[points[1][0], points[1][1]])
        text2.set_position((points[1][0], points[1][1]))
        text2.set_text('dist: %.3f' % dists[1])
        plot2[0].set_data([sample_point[i][0], points[1][0]], [sample_point[i][1], points[1][1]])
        point3.set_offsets(np.c_[points[2][0], points[2][1]])
        text3.set_position((points[2][0], points[2][1]))
        text3.set_text('dist: %.3f' % dists[2])
        plot3[0].set_data([sample_point[i][0], points[2][0]], [sample_point[i][1], points[2][1]])
        plot5[0].set_data([sample_point[i][0], obs_center[0][0]], [sample_point[i][1], obs_center[0][1]])
        plot6[0].set_data([sample_point[i][0], obs_center[1][0]], [sample_point[i][1], obs_center[1][1]])
        plot7[0].set_data([sample_point[i][0], obs_center[2][0]], [sample_point[i][1], obs_center[2][1]])
    
    plt.pause(0.05)

