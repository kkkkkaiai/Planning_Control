import matplotlib.pyplot as plt
import numpy
import math
import png
import numpy as np

def dist2d(point1, point2):
    """
    Euclidean distance between two points
    :param point1:
    :param point2:
    :return:
    """

    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]

    dist2 = (x1 - x2)**2 + (y1 - y2)**2

    return math.sqrt(dist2)


def png_to_ogm(filename, normalized=False, origin='lower'):
    """
    Convert a png image to occupancy data.
    :param filename: the image filename
    :param normalized: whether the data should be normalised, i.e. to be in value range [0, 1]
    :param origin:
    :return:
    """
    r = png.Reader(filename)
    img = r.read()
    img_data = list(img[2])

    out_img = []
    bitdepth = img[3]['bitdepth']

    for i in range(len(img_data)):

        out_img_row = []

        for j in range(len(img_data[0])):
            if j % img[3]['planes'] == 0:
                if normalized:
                    out_img_row.append(img_data[i][j]*1.0/(2**bitdepth))
                else:
                    out_img_row.append(img_data[i][j])

        out_img.append(out_img_row)

    if origin == 'lower':
        out_img.reverse()

    return out_img


def plot_path(path, color='y', linestyle=None, label=None):
    start_x, start_y = path[0]
    goal_x, goal_y = path[-1]

    # plot path
    path_arr = numpy.array(path)
    plt.plot(path_arr[:, 0], path_arr[:, 1], c=color, linestyle=linestyle, label=label)

    # plot start point
    plt.plot(start_x, start_y, 'ro')

    # plot goal point
    plt.plot(goal_x, goal_y, 'go')

def get_movements_4n():
    """
    Get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0)]


def get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]
    
def calc_ref_trajectory_in_T_step(robot, controller, ref_path, NX=3, dt=0.1, d_dist=0.1):
    """
    calc referent trajectory in T steps: [x, y, yaw]
    using the current velocity, calc the T points along the reference path
    :param robot: robot system
    :param controller: receding horizon controller
    :param ref_path: reference path: [x, y, yaw]
    :param sp: speed profile (designed speed strategy)
    :return: reference trajectory
    """
    state, vel = robot.get_position(), robot.get_velocity()
    T = controller.get_horizon()
    noninal_T = T+1
    traj_ref = np.zeros((NX, noninal_T))
    length = ref_path.length
    ind, _ = ref_path.nearest_index(state)

    traj_ref[0, 0] = ref_path.cx[ind]
    traj_ref[1, 0] = ref_path.cy[ind]
    traj_ref[2, 0] = ref_path.cyaw[ind]

    dist_move = 0.05

    for i in range(1, noninal_T):
        dist_move += abs(vel) * dt

        ind_move = int(round(dist_move / d_dist))
        index = min(ind + ind_move, length - 1)

        traj_ref[0, i] = ref_path.cx[index]
        traj_ref[1, i] = ref_path.cy[index]
        traj_ref[2, i] = ref_path.cyaw[index]

    return np.asarray(traj_ref)