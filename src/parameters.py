'''This file holds all of our parameters.
Access these by from parameters import *
'''
import numpy as np

SHOW_PLOTS = True

T = 8       # horizon length
DT = 0.2    # time increment
dl = 1.0    # course tick
MAX_TIME = 200.0
TARGET_SPEED = 10.0 / 3.6

MIN_V_Abs = -20.0 / 3.6
MAX_V_Abs = 30.0 / 3.6
MIN_V = -10.0 / 3.6
MAX_V = 10.0 / 3.6
MIN_DELTA = -np.deg2rad(45.0)
MAX_DELTA = np.deg2rad(45.0)
MAX_ACCEL = 1.0
MAX_DSTEER = np.deg2rad(30.0)
DIST_TO_GOAL = 20

L = 2.5     # Car length from rear to front wheel axles

#### Driving Cost Matrices ####
Q = np.diag([1.0, 1.0, 0.25, 0.5]) # State Cost Matrix
Qf = np.diag([1.0, 1.0, 0.25, 0.5]) # Final State Cost Matrix
R = np.diag([0.1, 0.1]) # Control Input Cost Matrix
Rd = np.diag([0.1, 0.5]) # Control Input Difference Cost Matrix

Q_park = np.diag([2.0, 2.0, 5.0, 0.1]) # State Cost Matrix
Qf_park = np.diag([2.0, 2.0, 5.0, 0.1]) # Final State Cost Matrix
R_park = np.diag([0.1, 0.1]) # Control Input Cost Matrix
Rd_park = np.diag([0.1, 1.0]) # Control Input Difference Cost Matrix