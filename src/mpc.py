'''Model Predictive Controller'''
import cvxpy
import numpy as np

from util import *

def linear_mpc():
    """ Linear MPC - LTV system """
    pass

    z = cvxpy.Variable((n, T + 1))
    u = cvxpy.Variable((m, T))

    objective_func = 0.0
    constraint = []

    # Populate Objective/Cost Function and Constraints


    # Problem
    problem = cvxpy.Problem(cvxpy.Minimize(objective_fun), constraint)
    problem.solve(solver= , verbose= )


    opt_control = control_input(0,0)
    opt_state = vehicle_state(0,0,0,0)
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
