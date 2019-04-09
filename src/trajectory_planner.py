'''Simulates a path planner by generating desired (x, y, phi, k)'''

import numpy as np
import math

from .spline_interpolation import *

def generate_trajectory(xlist, ylist):
    '''Returns a reference trajectory (x, y, phi, k)
        inputs: xlist, ylist are lists of waypoint coordinates
    '''
    ds = 1  # Let ds be the step-size of the trajectory
    sp = Spline2D(xlist, ylist)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, rphi, rk = [], [], [], []
    for index in s:
        ix, iy = sp.calc_position(index)
        rx.append(ix)
        ry.append(iy)
        rphi.append(sp.calc_phi(index))
        rk.append(sp.calc_curvature(index))
    return rx, ry, rphi, rk

def generate_speed_profile(rx, ry, rphi, target_speed):
    return [target_speed] * len(rx)

def calc_ref_trajectory():
    pass

def calc_nearest_index():
    pass