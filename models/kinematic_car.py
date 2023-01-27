from model import *

class KinematicCarDynamics:
    @staticmethod
    def forward_dynamics(x, u, timestep):
        """Return updated state in a form of `np.ndnumpy`"""
        l = 0.1
        x_next = np.ndarray(shape=(4,), dtype=float)
        x_next[0] = x[0] + x[2] * math.cos(x[3]) * timestep
        x_next[1] = x[1] + x[2] * math.sin(x[3]) * timestep
        x_next[2] = x[2] + u[0] * timestep
        x_next[3] = x[3] + x[2] * math.tan(u[1]) / l * timestep
        return x_next

    @staticmethod
    def forward_dynamics_opt(timestep):
        """Return updated state in a form of `ca.SX`"""
        l = 0.1
        x_symbol = ca.SX.sym("x", 4)
        u_symbol = ca.SX.sym("u", 2)
        x_symbol_next = x_symbol[0] + x_symbol[2] * ca.cos(x_symbol[3]) * timestep
        y_symbol_next = x_symbol[1] + x_symbol[2] * ca.sin(x_symbol[3]) * timestep
        v_symbol_next = x_symbol[2] + u_symbol[0] * timestep
        theta_symbol_next = x_symbol[3] + x_symbol[2] * ca.tan(u_symbol[1]) / l * timestep
        state_symbol_next = ca.vertcat(x_symbol_next, y_symbol_next, v_symbol_next, theta_symbol_next)
        return ca.Function("dubin_car_dynamics", [x_symbol, u_symbol], [state_symbol_next])

class KinematicCarSystem(System):
    def get_state(self):
        return self._state._x

    def update(self, unew):
        xnew = self._dynamics.forward_dynamics(self.get_state(), unew, 0.1)
        self._state._x = xnew
        self._time += 0.1

    def logging(self, logger):
        logger._xs.append(self._state._x)
        logger._us.append(self._state._u)