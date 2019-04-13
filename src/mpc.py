'''Model Predictive Controller'''
import cvxpy
import numpy as np

from util import *


#### Driving Cost Matrices ####
Q = np.diag(1.0, 1.0, 1.0, 1.0) # State Cost Matrix
Qf = Q # Final State Cost Matrix
R = np.diag(1.0, 1.0) # Control Input Cost Matrix
Rd = np.diag(1.0, 1.0) # Control Input Difference Cost Matrix

#### Parking Cost Matrices ####



def predict_motion(z0, opt_a, opt_d, zref):
    """Predicts future motion of vehicle at operational point"""
    # Initialize zbar
    zbar = np.zeros_like(zref)
    # First instance of zbar is the operational point
    zbar[:,0] = x0

    # update zbar with future predctions of state based on motion
    state = vehicle_state(x=z0[0], y=z0[1], phi=z0[2], v=z0[3])
    for i in range(1,T+1):
        vehicle_state = update_vehicle_state(vehicle_state, opt_a[i], opt_d[i])
        zbar[0,i] = state.x
        zbar[1,i] = state.y
        zbar[2,i] = state.v
        zbar[3,i] = state.phi

    return zbar


def linear_mpc():
    """ Linear MPC"""
    pass

    z = cvxpy.Variable((n, T + 1))
    u = cvxpy.Variable((m, T))

    objective_func = 0.0
    constraint = []

    # Populate Objective/Cost Function and Constraints
    for t in range(T):
        # Obj/Cost Function Term 1
        if t > 0:
            objective_func += cvxpy.quad_form(zref[:,t] - z[:,t], Q)
        # Obj/Cost Function Term 3
        objective_func += cvxpy.quad_form(u[:,t], R)
        # Obj/Cost Function Term 4
        if t < (T - 1):
            objective_func += cvxpy.quad_form(u[:,t+1] - u[:,t], Rd)
    # Obj/Cost Function Term 2
    objective_fun += cvxpy.quad_form(zref[:,T] - x[:,T], Qf)

    
    # Problem
    problem = cvxpy.Problem(cvxpy.Minimize(objective_fun), constraint)
    problem.solve(solver=ECOS , verbose=False )


    opt_control = control_input(a=0.0, delta=0.0)
    opt_state = vehicle_state(x=0.0, y=0.0, v=0.0,phi =0.0)
    # Solved Problem Results
    if prop.status == cvxpy.OPTIMAL or prop.status == cvxpy.OPTIMAL_INACCURATE:
        #populate results
        opt_control.a =
        opt_control.delta = 
        
        opt_state.x = 
        opt_state.y =
        opt_state.v =
        opt_state.phi =
        

    return opt_control, opt_state
