import numpy as np
from scipy.interpolate import splprep, splev

# Define control points
x = [0, 1, 2, 3, 4]
y = [0, 3, 1, 2, 1]

# Fit a B-spline curve through the control points
tck, u = splprep([x, y], s=0)

# Evaluate the B-spline curve at n points
n_points = 100
u_eval = np.linspace(0, 1, n_points)
x_bspline, y_bspline = splev(u_eval, tck)

# Plot the B-spline curve
import matplotlib.pyplot as plt

plt.plot(x_bspline, y_bspline)
plt.plot(x, y, 'ro')
plt.show()
