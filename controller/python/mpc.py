import datetime

import numpy as np
import casadi as ca


class NmpcOptimizerParam:
    def __init__(self) -> None:
        self.horizon = 10
        self.cbf_horizon = 6
        self.mat_Q = np.diag([1, 1, 2])
        self.mat_R = np.diag([1, 0.1])
        self.mat_Rd = np.diag([1, 0.1])
        self.mat_Jerk = np.diag([1, 0.1])
        self.terminal_weight = 10
        self.magin_dist = 0.3

class PlantParam:
    def __init__(self) -> None:
        self.amin = -0.5
        self.amax = 0.5
        self.omegamin = -0.5
        self.omegamax = 0.5
        self.jerkmin = -1.0
        self.jerkmax = 1.0
        self.omegadotmin = -0.5
        self.omegadotmax = 0.5


class NmpcOptimizer:
    '''
    Cost:
    1. reference trajectory tracking
    2. input stage
    3. input smoothness
    Constraints:
    1. initial condition
    2. input
    3. input derivative(jerk)
    4. dynamics
    '''
    def __init__(self, dynamics_opt, 
                 opt_param=NmpcOptimizerParam(), 
                 plant_param=PlantParam()) -> None:
        self._opt_param = opt_param
        self._plant_param = plant_param
        self._variables = {}
        self._dynamics_opt = dynamics_opt
        self._costs = {'reference_trajectory_tracking': 0, \
                       'input_stage': 0, \
                       'input_smoothness': 0}
        self._cost_time = []
        
    def set_state(self, state):
        self._state = state
    
    def initial_variables(self):
        '''
        state_x: x, y, theta, v
        '''
        self._variables["x"] = self.opti.variable(3, self._opt_param.horizon + 1)
        self._variables["u"] = self.opti.variable(2, self._opt_param.horizon)

    def add_initial_condition_constraint(self):
        self.opti.subject_to(self._variables["x"][:, 0] == self._state._x)

    def add_input_constraint(self):
        for i in range(self._opt_param.horizon):
            # input constraints
            self.opti.subject_to(self._variables["u"][0, i] <= self._plant_param.amax)
            self.opti.subject_to(self._plant_param.amin <= self._variables["u"][0, i])
            self.opti.subject_to(self._variables["u"][1, i] <= self._plant_param.omegamax)
            self.opti.subject_to(self._plant_param.omegamin <= self._variables["u"][1, i])

    def add_input_derivative_constraint(self):
        for i in range(self._opt_param.horizon - 1):
            # input constraints
            self.opti.subject_to(self._variables["u"][0, i + 1] - self._variables["u"][0, i] <= self._plant_param.jerkmax)
            self.opti.subject_to(self._variables["u"][0, i + 1] - self._variables["u"][0, i] >= self._plant_param.jerkmin)
            self.opti.subject_to(self._variables["u"][1, i + 1] - self._variables["u"][1, i] <= self._plant_param.omegadotmax)
            self.opti.subject_to(self._variables["u"][1, i + 1] - self._variables["u"][1, i] >= self._plant_param.omegadotmin)
        self.opti.subject_to(self._variables["u"][0, 0] - self._state._u[0] <= self._plant_param.jerkmax)
        self.opti.subject_to(self._variables["u"][0, 0] - self._state._u[0] >= self._plant_param.jerkmin)
        self.opti.subject_to(self._variables["u"][1, 0] - self._state._u[1] <= self._plant_param.omegadotmax)
        self.opti.subject_to(self._variables["u"][1, 0] - self._state._u[1] >= self._plant_param.omegadotmin)

    def add_dynamics_constraint(self):
        for i in range(self._opt_param.horizon):
            self.opti.subject_to(
                self._variables["x"][:, i + 1] == self._dynamics_opt(self._variables["x"][:, i], self._variables["u"][:, i])
            )

    def add_reference_trajectory_tracking_cost(self, reference_trajectory):
        self._costs["reference_trajectory_tracking"] = 0
        # tracking cost
        for i in range(self._opt_param.horizon - 1):
            x_diff = self._variables["x"][:, i] - reference_trajectory[i, :]
            self._costs["reference_trajectory_tracking"] += ca.mtimes(x_diff.T, ca.mtimes(self._opt_param.mat_Q, x_diff))
        x_diff = self._variables["x"][:, -1] - reference_trajectory[-1, :]
        # terminal cost
        self._costs["reference_trajectory_tracking"] += self._opt_param.terminal_weight * ca.mtimes(
            x_diff.T, ca.mtimes(self._opt_param.mat_Q, x_diff)
        )

    def add_input_stage_cost(self, param):
        self._costs["input_stage"] = 0
        for i in range(param.horizon):
            self._costs["input_stage"] += ca.mtimes(
                self.variables["u"][:, i].T, ca.mtimes(param.mat_R, self.variables["u"][:, i])
            )

    def add_input_smoothness_cost(self, param):
        self._costs["input_smoothness"] = 0
        for i in range(param.horizon - 1):
            self._costs["input_smoothness"] += ca.mtimes(
                (self.variables["u"][:, i + 1] - self.variables["u"][:, i]).T,
                ca.mtimes(param.mat_dR, (self.variables["u"][:, i + 1] - self.variables["u"][:, i])),
            )

    def setup(self, system, ref_traj):
        self.set_state(system.get_state())
        self.opti = ca.Opti()
        self.initial_variables()
        self.add_initial_condition_constraint()
        self.add_input_constraint()
        self.add_dynamics_constraint()
        self.add_reference_trajectory_tracking_cost(ref_traj)


    def solve_nlp(self):
        cost = 0
        for cost_name in self._costs:
            cost += self._costs[cost_name]

        self.opti.minimize(cost)
        option = {"verbose": False, "ipopt.print_level": 0, "print_time": 0}
        start_timer = datetime.datetime.now()
        self.opti.solver("ipopt", option)
        opt_sol = self.opti.solve()
        end_timer = datetime.datetime.now()
        delta_timer = end_timer - start_timer
        self._cost_time.append(delta_timer.total_seconds())
        # print("solver time: ", delta_timer.total_seconds())
        return opt_sol

