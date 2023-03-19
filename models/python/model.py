import numpy as np
import casadi as ca
import math
class System:
    def __init__(self, time=0.0, state=None, geometry=None, dynamics=None, simu_time=0.1):
        self._time = time
        self._simu_time = simu_time
        self._state = state
        self._geometry = geometry
        self._dynamics = dynamics