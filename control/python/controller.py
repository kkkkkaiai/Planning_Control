import numpy as np
from control.python.mpc import NmpcOptimizer, NmpcOptimizerParam

class NmpcController:
    def __init__(self, dynamic, opt_param=NmpcOptimizerParam()) -> None:
        self._param = opt_param
        self._optimizer = NmpcOptimizer(dynamic.forward_dynamics_opt(0.1), opt_param)

    def get_horizon(self):
        return self._param.horizon
    
    def generate_control_input(self, system, local_trajectory):
        self._optimizer.setup(system, local_trajectory)
        self._opt_sol = self._optimizer.solve_nlp()
        return self._opt_sol.value(self._optimizer._variables["u"][:, 0])
    
    def get_optimal_trajectory(self):
        if self._opt_sol:
            return self._opt_sol.value(self._optimizer._variables["x"])
        else:
            raise Exception("No solution found yet")
    
    def logging(self, logger):
        logger._xtrajs.append(self._opt_sol.value(self._optimizer._variables["x"]).T)
        logger._utrajs.append(self._opt_sol.value(self._optimizer._variables["u"]).T)