from model import *

class DifferentalCarDynamics:
    @staticmethod
    def forward_dynamics(x, u, timestep):
        """Return updated state in a form of `np.ndnumpy`"""
        l = 0.1
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
        return ca.Function("dubin_car_dynamics", [x_symbol, u_symbol], [state_symbol_next])

class DifferentialCarSystem(System):
    def get_state(self):
        return self._state._x

    def update(self, unew):
        xnew = self._dynamics.forward_dynamics(self.get_state(), unew, 0.1)
        self._state._x = xnew
        self._time += 0.1

    def logging(self, logger):
        logger._xs.append(self._state._x)
        logger._us.append(self._state._u)