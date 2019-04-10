''' Linearize and Discritize Vehicle Model, update State '''

from util import *
import numpy as np
import math
from scipy.linalg import expm

def linearize_kinematics_model_approx(v, phi, delta):
    ''' Linearize and Discritize the Kinematics Vehicle Model '''

    A = np.zeros((n,n))
    B = np.zeros((n,m))
    C = np.zeros(n)

    # Calculate A Matrix
    A[0,0] = 1.0
    A[0,2] = math.cos(phi) * dt
    A[0,3] = -v * math.sin(phi) * dt
    A[1,1] = 1.0
    A[1,2] = math.sin(phi) * dt
    A[1,3] = v * math.cos(phi) * dt
    A[2,2] = 1.0
    A[3,2] = math.tan(delta) * dt / L
    A[3,3] = 1.0
    
    # Calculate B Matrix
    B[2,0] = dt
    B[3,1] = v * dt / (L * math.cos(delta)**2)

    # Calculate C Matrix
    C[0] = v * phi * dt * math.sin(phi)
    C[1] = -v * phi * dt * math.cos(phi)
    C[3] = -v * delta * dt / (L * math.cos(delta)**2)


    return A, B, C
'''
def linearize_kinematics_model_exact(v, phi, delta):
    ''' Linearize and Discritize the Kinamatics Vehicle Model Exact'''

    dA = np.zeros((n,n))
    dB = np.zeros((n,m))
    C = np.zeros(n)

    # Calculate A' Matrix
    dA[0,2] = math.cos(phi)
    dA[0,3] = -v * math.sin(phi)
    dA[1,2] = math.sin(phi)
    dA[1,3] = v * math.cos(phi)
    dA[3,2] = math.tan(delta) / L

    # Calculate B' Matrix
    dB[2,0] = 1.0
    dB[3,1] = v / (L * math.cos(delta)**2)

    # Calculate A Matrix
    A = expm(dA*dt)
    print('I got here at least')
    # Calculate B Matrix
    B = np.linalg.inv(dA) * (A - eye(n,n)) * dB
    print('I also got here?')
    # Calculate C Matrix
    C[0] = v * phi * dt * math.sin(phi)
    C[1] = -v * phi * dt * math.cos(phi)
    C[3] = -v * delta * dt / (L * math.cos(delta)**2)

    return A, B, C
'''