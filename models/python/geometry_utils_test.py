"""
@author: Kai Chen
"""
from geometry_utils import SuperEllipse
import numpy as np
import matplotlib.pyplot as plt

def test_ellipse():
    s = SuperEllipse([2.5, 1.8], 0.5, None, 45, 20)
    s.plot()
    plt.show()

test_ellipse()