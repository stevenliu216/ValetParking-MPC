'''Simulation loop'''
import numpy as np
import matplotlib.pyplot as plt

from .parameters import DT, MAX_TIME
from .path_planner import *
from .mpc import *
from .util import *

def update_state(state):
    return state

def check_goal(goal):
    return False

def simulate(test_track, speed, dl):
    '''
    inputs:
    test_track: [0] is x
                [1] is y
                [2] is phi
                [3] is v
    speed: speed profile
    dl: tick [m]

    returns:
    t: time (list for plotting)
    x: x position
    y: y position
    phi: heading
    v: velocity
    a: acceleration
    delta: steering angle
    '''

    # Initialize the state
    state = vehicle_state(test_track[0][0], test_track[1][0], 0.0, test_track[2][0])
    goal = [test_track[0][-1], test_track[1][-1]]
    
    path_planner = PathPlanner(state, test_track, speed, 0)
    path_planner.index = path_planner.calc_nearest_index()
    # smooth_yaw -> Don't do for now

    # time loop
    time = 0.0
    
    while (time <= MAX_TIME):
        # calc_ref_trajectory every time loop
        path_planner.calc_ref_trajectory()
        # iterative linear mpc every time loop
        linear_mpc()
        # update state every time loop
        state = update_state(state)
        # update the lists for book-keeping

        # terminate if goal is reached (checkgoal())
        if check_goal(goal):
            break

        # increment time
        time += DT
    
    # book-keeping for plots
    t, x, y, phi, v, a, delta = [], [], [], [], [], [], []
    return t, x, y, phi, v, a, delta