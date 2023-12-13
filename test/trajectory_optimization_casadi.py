import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# ref website: https://web.casadi.org/blog/ocp/

N = 100

opti = ca.Opti()

X = opti.variable(2, N+1) # the first row is position, the second row is speed
pos = X[0, :]
speed = X[1, :]
U = opti.variable(1, N)
T = opti.variable()

opti.minimize(T)

f = lambda x, u: ca.vertcat(x[1], u-x[1]) # dx/dt = f(x, u)

dt = T / N
for i in range(N):
    # Runge-Kutta 4 integration
    k1 = f(X[:, i], U[:, i])
    k2 = f(X[:, i] + dt / 2 * k1, U[:, i])
    k3 = f(X[:, i] + dt / 2 * k2, U[:, i])
    k4 = f(X[:, i] + dt * k3, U[:, i])
    x_next = X[:, i] + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    opti.subject_to(X[:, i + 1] == x_next)

limit = lambda pos: 1-ca.sin(2*ca.pi*pos)/2

opti.subject_to(limit(pos) >= speed)
# limit the input
opti.subject_to(U <= 1)
opti.subject_to(U >= 0)

# start at position 0
opti.subject_to(X[:, 0] == ca.vertcat(0, 0))
# end at position 1
opti.subject_to(X[0, -1] == 1)


# time must be positive
opti.subject_to(T >= 0)

# set initial guess
opti.set_initial(T, 0.8)
opti.set_initial(speed, 1)

# set ipopt parameter
opti.solver('ipopt', {'expand': True, 'ipopt.max_iter': 1000, 'ipopt.print_level': 0, 'print_time': 1})

sol = opti.solve()

# plot the limit speed
limit_index = sol.value(pos)
time_index = np.linspace(0, sol.value(T), N+1)
plt.plot(time_index, limit(limit_index), ls='--', color='red', label='limit speed')

# plot the trajectory
plt.plot(time_index, sol.value(X[0, :]), color='orange', label='position')
plt.plot(time_index, sol.value(X[1, :]), color='blue', label='speed')

# plot the input
plt.plot(time_index[:-1], sol.value(U[0, :]), color='black', label='input')

# create a new ax and plot the jacobian of opti.g and opti.x
fig = plt.figure()
ax = fig.gca()
value = sol.value(ca.jacobian(opti.g, opti.x))
ax.spy(value, markersize=1, color='blue', label='jacobian')

plt.legend()
plt.show()

