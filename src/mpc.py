'''Model Predictive Controller'''
import math
import cvxpy
import numpy as np
from scipy.linalg import expm

from .util import *
from .parameters import *

#### Parking Cost Matrices ####

def update_vehicle_state(state, a, delta):
    '''Update Vehicle State based on mpc control output'''
    state.x = state.x + state.v * math.cos(state.phi) * DT
    state.y = state.y + state.v * math.sin(state.phi) * DT
    
    state.v = state.v + a * DT
    # Limit vehicle speed to plausible max/min
    state.v = max(min(state.v, MAX_V), MIN_V)

    # Limit steering angle to maximum value
    delta = max(min(delta, MAX_DELTA), -MIN_DELTA)
    state.phi = state.phi + (state.v * math.tan(delta) / L) * DT
    
    return state

def linearize_kinematics_model_approx(v, phi, delta):
    ''' Linearize and Discritize the Kinematics Vehicle Model '''

    A = np.zeros((4,4))
    B = np.zeros((4,2))
    C = np.zeros(4)

    # Calculate A Matrix
    A[0,0] = 1.0
    A[0,2] = math.cos(phi) * DT
    A[0,3] = -v * math.sin(phi) * DT
    A[1,1] = 1.0
    A[1,2] = math.sin(phi) * DT
    A[1,3] = v * math.cos(phi) * DT
    A[2,2] = 1.0
    A[3,2] = math.tan(delta) * DT / L
    A[3,3] = 1.0
    
    # Calculate B Matrix
    B[2,0] = DT
    B[3,1] = v * DT / (L * math.cos(delta)**2)

    # Calculate C Matrix
    C[0] = v * phi * DT * math.sin(phi)
    C[1] = -v * phi * DT * math.cos(phi)
    C[3] = -v * delta * DT / (L * math.cos(delta)**2)


    return A, B, C



def predict_motion(z0, opt_a, opt_d, zref):
    """Predicts future motion of vehicle at operational point"""
    # Initialize zbar
    zbar = np.zeros_like(zref)
    # First instance of zbar is the operational point
    zbar[:,0] = z0

    # update zbar with future predctions of state based on motion
    state = vehicle_state(x=z0[0], y=z0[1], v=z0[2], phi=z0[3])
    for i in range(1,T+1):
        state = update_vehicle_state(state, opt_a, opt_d)
        zbar[0,i] = state.x
        zbar[1,i] = state.y
        zbar[2,i] = state.v
        zbar[3,i] = state.phi

    return zbar

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def linear_mpc(zref, zbar, z0, dref):
    """ Linear MPC"""
    

    z = cvxpy.Variable((4, T + 1))
    u = cvxpy.Variable((2, T))

    objective_func = 0.0
    constraints = []

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
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

        A, B, C = linearize_kinematics_model_approx(zbar[2,t], zbar[3,t], dref)
        constraints += [z[:,t+1] == A * z[:, t] + B * u[:, t] + C]
    
    # Obj/Cost Function Term 2
    objective_func += cvxpy.quad_form(zref[:,T] - z[:,T], Qf)
    constraints += [z[:, 0] == z0]
    constraints += [z[2, :] <= MAX_V]
    constraints += [z[2, :] >= MIN_V]
    constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
    constraints += [cvxpy.abs(u[1, :]) <= MAX_DELTA]
    
    # Problem
    problem = cvxpy.Problem(cvxpy.Minimize(objective_func), constraints)
    problem.solve(solver=cvxpy.ECOS, verbose=False)


    opt_control = control_input(a=0.0, delta=0.0)
    opt_state = vehicle_state(x=0.0, y=0.0, v=0.0, phi =0.0)
    # Solved Problem Results
    if problem.status == cvxpy.OPTIMAL or problem.status == cvxpy.OPTIMAL_INACCURATE:
        #populate results
        print('solver status: {}'.format(problem.status))
        opt_control.a = get_nparray_from_matrix(u.value[0, :])
        opt_control.delta = get_nparray_from_matrix(u.value[1, :])
        
        opt_state.x = get_nparray_from_matrix(z.value[0, :])
        opt_state.y =get_nparray_from_matrix(z.value[1, :])
        opt_state.v = get_nparray_from_matrix(z.value[2, :])
        opt_state.phi = get_nparray_from_matrix(z.value[3, :])
        

    return opt_control, opt_state
