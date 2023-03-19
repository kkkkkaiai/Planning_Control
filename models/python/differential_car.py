from models.python.model import *
from matplotlib.patches import Circle
from copy import copy
import numpy as np

class DifferentalCarDynamics:
    @staticmethod
    def forward_dynamics(x, u, timestep):
        """Return updated state in a form of `np.ndnumpy`"""
        x_next = np.ndarray(shape=(3,), dtype=float)
        x_next[0] = x[0] + u[0] * math.cos(x[2]) * timestep
        x_next[1] = x[1] + u[0] * math.sin(x[2]) * timestep
        x_next[2] = x[2] + u[1] * timestep
        return x_next

    @staticmethod
    def forward_dynamics_opt(timestep):
        """Return updated state in a form of `ca.SX`"""
        l = 0.1
        x_symbol = ca.SX.sym("x", 3)
        u_symbol = ca.SX.sym("u", 2)
        x_symbol_next = x_symbol[0] + u_symbol[0] * ca.cos(x_symbol[2]) * timestep
        y_symbol_next = x_symbol[1] + u_symbol[0] * ca.sin(x_symbol[2]) * timestep
        theta_symbol_next = x_symbol[2] + u_symbol[1] * timestep
        state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, theta_symbol_next)
        return ca.Function("differential_car_dynamics", [x_symbol, u_symbol], [state_symbol_next])

    @staticmethod
    def nominal_safe_controller(x, timestep, amin, amax):
        """Return updated state using nominal safe controller in a form of `np.ndnumpy`"""
        u_nom = np.zeros(shape=(2,))
        u_nom[0] = np.clip(-x[2] / timestep, amin, amax)
        return DifferentalCarDynamics.forward_dynamics(x, u_nom, timestep), u_nom

class DifferrentialCarStates:
    def __init__(self, x, u=np.array([0.0, 0.0])):
        '''
        x: [x, y, theta], ndarray
        u: [v, omega], ndarray
        '''
        self._x = x
        self._last_x = copy(x)
        self._u = u

    def translation(self):
        return np.array([[self._x[0]], [self._x[1]]])

    def rotation(self):
        return np.array(
            [
                [math.cos(self._x[2]), -math.sin(self._x[2])],
                [math.sin(self._x[2]), math.cos(self._x[2])],
            ]
        )
    
    def velocity(self, dt):
        dx = self._x[0]-self._last_x[0]
        dy = self._x[1]-self._last_x[1]
        # calculate the direction of the velocity
        ratio = 1
        yaw = np.radians(self._x[2])
        v_yaw = math.atan2(dy, dx)
        error = v_yaw - yaw
        if error < -math.pi:
            error += 2*math.pi
        elif error > math.pi:
            error -= 2*math.pi
        error = abs(error)
        if error > math.pi/2:
            ratio = -1

        return ratio * math.sqrt((dx/dt)**2 + (dy/dt)**2)
    

class DifferentialCircleGeomery:
    def __init__(self, state, radius=0.3) -> None:
        self._radius = radius
        self._position = (state[0], state[1], state[2])
        self._circle = None

    def equiv_rep(self):
        pass
    
    def get_plot_patch(self, position, resolusion):
        if self._circle is None:
            self._circle = Circle((position[0]/resolusion, position[1]/resolusion), \
                                   self._radius/resolusion, fill=False, color="red")
        else:
            self._circle.set_center((position[0]/resolusion, position[1]/resolusion))
        return self._circle

class DifferentialCarSystem(System):
    def get_state(self):
        return self._state
    
    def get_position(self):
        return self._state._x

    def get_velocity(self):
        return self._state.velocity(self._simu_time)

    def get_dynamics(self):
        return self._dynamics

    def update_plot(self, resolusion=1):
        return self._geometry.get_plot_patch(self.get_position(), resolusion)

    def update(self, unew):
        xnew = self._dynamics.forward_dynamics(self.get_position(), unew, self._simu_time)
        self._state._last_x = copy(self._state._x)
        self._state._x = xnew
        self._time += 0.1

    def logging(self, logger):
        logger._xs.append(self._state._x)
        logger._us.append(self._state._u)
