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
    start_pos, end_pos = (0, 0), (20.0, 0)
    robot = DifferentialCarSystem(
        state=DifferrentialCarStates(np.array([start_pos[0], start_pos[1], 0.0]), np.array([0.0, 0.0])),
        geometry=DifferentialCircleGeomery(np.array([0.0, 0.0, 0.0])),
        dynamics=DifferentalCarDynamics()
    )
    controller = NmpcController(robot.get_dynamics(), NmpcOptimizerParam())
    # pM = PathManager()
    # pM.find_path((6.0, 3.0), (16.0, 16.0))
    # path = pM.find_gradient()
    # ref_path, sp = pM.spline_interpolate(path) # sp is speed profile

    x = np.linspace(0, 20, 100)
    y = np.zeros_like(x)
    ds = 0.1
    cx, cy, cyaw, ck, s = calc_spline_course(x, y, ds=ds)

    sp = calc_speed_profile(cx, cy, cyaw, 10)
    ref_path = PATH(cx, cy, cyaw, ck)
    print('Interpolated path length: ', ref_path.length)

    while True:
        ref_traj = calc_ref_trajectory_in_T_step(robot, controller, ref_path, dt=0.1)
        control_input = controller.generate_control_input(robot, ref_traj.T)
        robot.update(control_input)
        print(robot.get_position())

    plt.plot(x, y)
    plt.show()
    

    
    
    