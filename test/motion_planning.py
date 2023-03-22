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
from planner.path_generator.astar import *
from planner.trajectory_generator.spline_interpolate import *
from controller.python.controller import *
from utils.util import calc_ref_trajectory_in_T_step
from matplotlib.animation import FFMpegWriter

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
    interpolate_dist = 0.1
    ref_path, sp = pM.spline_interpolate(path, ds=interpolate_dist) # sp is speed profile
    reference_traj = None
    optimal_traj = None

    pM.plot_map()
    pM.plot_path()
    pM.plot_interpolated_path()
    fig = plt.gcf()
    plt.tight_layout()
    ax = plt.gca()
    ax.add_patch(robot.update_plot(resolution))
    metadata = dict(title='test', artist='Matplotlib',comment='test')
    writer = FFMpegWriter(fps=10, metadata=metadata)


    dis = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
    with writer.saving(fig, 'results/' + 'test.mp4', 300):
        achieved = False
        while not achieved:
            ref_traj = calc_ref_trajectory_in_T_step(robot, controller, ref_path, dt=0.1, d_dist=interpolate_dist)
            if reference_traj is None:
                (reference_traj,) = ax.plot(ref_traj[0, :]/resolution, ref_traj[1, :]/resolution, "-", color="blue", linewidth=1, markersize=2)
            else:
                reference_traj.set_data(ref_traj[0, :]/resolution, ref_traj[1, :]/resolution)
            control_input = controller.generate_control_input(robot, ref_traj.T)
            opt_traj = controller.get_optimal_trajectory()
            if optimal_traj is None:    
                (optimal_traj,) = ax.plot(opt_traj[0, :]/resolution, opt_traj[1, :]/resolution, "-", color="green", linewidth=1, markersize=2)
            else:
                optimal_traj.set_data(opt_traj[0, :]/resolution, opt_traj[1, :]/resolution)
            robot.update(control_input)
            robot.update_plot(resolution)
            dis = np.linalg.norm(np.array(end_pos) - np.array(robot.get_position()[:2]))
            if dis < 0.05:
                achieved = True
            print('ditance to target', dis)
            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
            plt.pause(0.0001) # pause a bit so that plots are updated
            plt.axis('off')
            writer.grab_frame()



    

    
    
    