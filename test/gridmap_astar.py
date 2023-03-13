import os, sys
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from envs.gridmap import OccupancyGridMap
from planning.path_generator.astar import a_star
from planning.trajectory_generator.spline_interpolate import *
from utils.util import *

gmap = OccupancyGridMap.from_png('envs/maps/example_map_occupancy.png', 0.05)

# set a start and an end node (in meters)
start_node = (6.0, 3.0)
goal_node = (16.0, 16.0)

# run A*
path, path_px = a_star(start_node, goal_node, gmap, movement='8N')
# path_arr = np.asarray(path) / 0.05
# print(path_arr)
# cx, cy, cyaw, ck, s = calc_spline_course(path_arr[:, 0], path_arr[:, 1], ds=0.1)
# sp = calc_speed_profile(cx, cy, cyaw, 10)
# ref_path = PATH(cx, cy, cyaw, ck)

gmap.plot()
if len(path) > 0:
    # plot resulting path in pixels over the map
    plot_path(path_px)
    # plt.plot(cx, cy)
else:
    print('Goal is not reachable')

    # plot start and goal points over the map (in pixels)
    start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
    goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

    plt.plot(start_node_px[0], start_node_px[1], 'ro')
    plt.plot(goal_node_px[0], goal_node_px[1], 'go')

plt.show()