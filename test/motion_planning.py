"""
author: Kai CHEN
mail: chenkai0130@outlook.com
license: MIT
"""

import sys, os
try:
    sys.path.append(os.getcwd())
except IndexError:
    pass

from models.python.differential_car import (
    DifferentalCarDynamics,
    DifferentialCircleGeomery,
    DifferentialCarSystem,
    DifferrentialCarStates
)
import numpy as np
import matplotlib.pyplot as plt
from gridmap_astar import PathManager
from planning.path_generator.astar import *
from planning.trajectory_generator.spline_interpolate import *
from control.python.controller import *
from utils.util import calc_ref_trajectory_in_T_step

if __name__ == '__main__':
    start_pos, end_pos = (6.0, 3.0), (16.0, 16.0)
    resolution = 0.05
    robot = DifferentialCarSystem(
        state=DifferrentialCarStates(np.array([start_pos[0], start_pos[1], 0.0]), np.array([0.0, 0.0])),
        geometry=DifferentialCircleGeomery(np.array([0.0, 0.0, 0.0])),
        dynamics=DifferentalCarDynamics()
    )
    controller = NmpcController(robot.get_dynamics(), NmpcOptimizerParam())
    
    # Path planning in grid map
    pM = PathManager(resolusion=resolution)
    pM.find_path(start_pos, end_pos)
    path = pM.find_gradient()
    front_repeat = np.repeat(path[0], 3)
    ref_path, sp = pM.spline_interpolate(path) # sp is speed profile
    pM.plot_map()
    pM.plot_path()
    pM.plot_interpolated_path()

    ax = plt.gca()
    ax.add_patch(robot.update_plot(resolution))
    dis = np.linalg.norm(np.array(end_pos) - np.array(start_pos))

    while dis > 0.001:
        ref_traj = calc_ref_trajectory_in_T_step(robot, controller, ref_path, dt=0.1)
        control_input = controller.generate_control_input(robot, ref_traj.T)
        robot.update(control_input)
        robot.update_plot(resolution)
        dis = np.linalg.norm(np.array(end_pos) - np.array(robot.get_position()[:2]))
        print(np.array(robot.get_position()[:2]))
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape' else None])
        plt.axis("equal")
        plt.pause(0.0001)

    

    
    
    