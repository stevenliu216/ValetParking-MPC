'''This file holds all of our parameters.
Access these by from parameters import *
'''
import numpy as np

SHOW_PLOTS = True

T = 5       # horizon length
DT = 0.2    # time increment
dl = 1.0    # course tick
MAX_TIME = 300.0
TARGET_SPEED = 10.0 / 3.6

MIN_V = -20.0 / 3.6
MAX_V = 15.0 / 3.6
MIN_DELTA = np.deg2rad(45.0)
MAX_DELTA = np.deg2rad(45.0)

L = 4.0