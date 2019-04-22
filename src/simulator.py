'''Simulation loop'''
import numpy as np
import matplotlib.pyplot as plt
import logging

from .parameters import *
from .path_planner import *
from .mpc import *
from .util import *

def simulate(test_track, speed):
    logging.info('Running within simulation now')
    # Initialize the state
    state = vehicle_state(test_track[0][0], test_track[1][0], 0.0, test_track[2][0])
    goal = [test_track[0][-1], test_track[1][-1]]
    opt_control = control_input(a=0.0, delta=0.0)

    path_planner = PathPlanner(test_track, speed, 0)
    path_planner.index = path_planner.calc_nearest_index(state)

    # time loop
    time = 0.0
    t = [time]
    
    # book-keeping for plots
    x, y, phi, v = [], [], [], []
    speed_profile = []
    phi_ref = []
    phi_error = []
    position_refx = []
    position_refy = []
    position_error = []

    a = 0.0
    delta = 0.0
    park_flag = False

    # temporary print
    np.set_printoptions(precision=1)
    while (time <= MAX_TIME):
        logging.info('\n===================================\ntime: {}'.format(time))
        # calc_ref_trajectory every time loop
        xref = path_planner.calc_ref_trajectory(state)
        logging.info('\nxref: \n{}'.format(xref))

        x0 = [state.x, state.y, state.v, state.phi]
        logging.info('\nx0: \nx:{:2f} y:{:2f} v:{:2f} phi:{:2f}'\
            .format(state.x, state.y, state.v, state.phi))

        if a is None or delta is None:
            a = 0.0
            delta = 0.0
        
        xbar = predict_motion(x0, a, delta, xref)
        logging.info('\nxbar: \n{}'.format(xbar))

        # see if close to parking
        park_flag = path_planner.get_park_flag(state)

        # iterative linear mpc every time loop
        opt_control, opt_state = linear_mpc(xref, xbar, x0, 0.0, park_flag)
        logging.info('\nMPC control: \n a: {}, \n delta: {}'.format(opt_control.a, opt_control.delta))
        logging.info('\nMPC states: \n x: {}, \n y:{}'.format(opt_state.x, opt_state.y))

        # update state every time loop
        a = opt_control.a[0]
        delta = opt_control.delta[0]
        state = update_vehicle_state(state, a, delta)

        # terminate if goal is reached (checkgoal())
        if path_planner.check_goal(state, goal):
            print('=========reached goal=========\n')
            logging.info('Reached goal.')
            logging.info('Desired goal: \nx:{:2f} y:{:2f}'.format(goal[0], goal[1]))
            logging.info('End configuration: \nx:{:2f} y:{:2f} v:{:2f} phi:{:2f}'\
                .format(state.x, state.y, state.v, state.phi))
                
            plt.close()
            break

        # increment time
        time += DT
        t.append(time)
        x.append(state.x)
        y.append(state.y)
        v.append(state.v)
        speed_profile.append(speed[path_planner.index])
        phi.append(state.phi)
        phi_ref.append(xref[3,0])
        phi_error.append(np.linalg.norm(xref[3,0]-state.phi))
        position_refx.append(xref[0,0])
        position_refy.append(xref[1,0])
        position_error.append(np.linalg.norm(np.asarray([state.x,state.y])-xref[0:2,0]))

        if SHOW_PLOTS:
            plt.subplot(2,3,1)
            plt.cla()
            if opt_state.x is not None:
                plt.plot(opt_state.x, opt_state.y, "c*", label="MPC")
            plt.plot(test_track[0], test_track[1], "--k", label="track")
            plt.plot(x, y, "xr", label="traj")
            plt.plot(xref[0, :], xref[1, :], "xb", label="xref")
            plt.plot(test_track[0][path_planner.index], test_track[1][path_planner.index], "go", markersize=6, label="target")
            
            plt.legend()
            plt.pause(0.001)

            plt.subplot(2,3,4)
            plt.plot(speed_profile, '-b')
            plt.title('Speed Profile')
            plt.plot(v, '-r')
            plt.pause(0.001)
            
            plt.subplot(2,3,2)
            plt.title('Heading Angle')
            plt.plot(phi, '--r', label='steering angle')
            plt.plot(phi_ref, '-b', label='reference angle')
            
            plt.subplot(2,3,5)
            plt.title('Heading Angle RMSE')
            plt.plot(phi_error, 'k')
            plt.ylim(-1,5)

            plt.subplot(2,3,3)
            plt.title('Trajectory zoomed in')
            plt.plot(x,y, "-rx", label = "Actual Position")
            plt.plot(position_refx,position_refy,"-bx", label = "Reference Position")

            plt.subplot(2,3,6)
            plt.title('Trajectory RMSE')
            plt.plot(position_error, 'k')
            plt.ylim(-1,5)

    return t, x, y, phi, v, a, delta