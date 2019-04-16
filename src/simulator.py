'''Simulation loop'''
import numpy as np
import matplotlib.pyplot as plt

from .parameters import *
from .path_planner import *
from .mpc import *
from .util import *

def update_vehicle_state(state, a, delta):
    '''Update Vehicle State based on mpc control output'''
    
    state.v = state.v + a * DT
    # Limit vehicle speed to plausible max/min
    state.v = max(min(state.v, MAX_V), MIN_V)

    # Limit steering angle to maximum value
    delta = max(min(delta, MAX_DELTA), -MIN_DELTA)
    state.phi = state.phi + (state.v * math.tan(delta) / L) * DT
    
    state.x = state.x + state.v * math.cos(state.phi) * DT
    state.y = state.y + state.v * math.sin(state.phi) * DT

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
    
    path_planner = PathPlanner(test_track, speed, 0)
    path_planner.index = path_planner.calc_nearest_index(state)
    # smooth_yaw -> Don't do for now

    # time loop
    time = 0.0
    t = [time]
    
    # book-keeping for plots
    x, y, phi, v, a, delta = [], [], [], [], [], []

    while (time <= MAX_TIME):
        # calc_ref_trajectory every time loop
        xref = path_planner.calc_ref_trajectory(state)

        # iterative linear mpc every time loop
        linear_mpc()
        # update state every time loop
        a = 0.5
        delta = 0.0
        state = update_vehicle_state(state, a, delta)
        # update the lists for book-keeping

        # terminate if goal is reached (checkgoal())
        if check_goal(goal):
            break

        # increment time
        time += DT
        t.append(time)
        x.append(state.x)
        y.append(state.y)
        v.append(state.v)
        phi.append(state.phi)

        if SHOW_PLOTS:
            plt.figure(3)
            plt.plot(test_track[0], test_track[1], "-b", label="track")
            plt.plot(x, y, "-m", label="traj")
            plt.plot(xref[0, :], xref[1, :], "ro", label="xref")
            plt.plot(test_track[0][path_planner.index], test_track[1][path_planner.index], "gx", label="target")
            plt.axis("equal")
            plt.pause(0.001)
            

    return t, x, y, phi, v, a, delta