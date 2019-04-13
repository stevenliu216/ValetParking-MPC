'''Simulates a path planner'''

import numpy as np
import math

class PathPlanner:
    def __init__(self, state, test_track, speed, index):
        self.state = state
        self.cx = test_track[0]
        self.cy = test_track[1]
        self.cphi = test_track[2]
        self.ck = test_track[3]
        self.speed = speed
        self.index = index

        self.xref = None
        self.DT = 0.1
        self.T = 5
        self.dl = 1
    
    def calc_ref_trajectory(self):
        '''Simulates an online path planner by returning x_ref'''
        T = self.T
        xref = np.zeros((4, T+1))
        dref = np.zeros((1, T+1))
        n = len(self.cx)

        ind, _ = self.calc_nearest_index()
        if self.index >= ind:
            ind = self.index

        xref[0,0] = self.cx[ind]
        xref[1,0] = self.cy[ind]
        xref[2,0] = self.speed[ind]
        xref[3,0] = self.cphi[ind]
        dref[0,0] = 0.0

        travel = 0.0
        for i in range(T+1):
            travel += abs(self.state.v) * self.DT
            dind = int(round(travel / self.dl))

            if (ind + dind) < n:
                xref[0, i] = self.cx[ind + dind]
                xref[1, i] = self.cy[ind + dind]
                xref[2, i] = self.speed[ind + dind]
                xref[3,i ] = self.cphi[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = self.cx[n - 1]
                xref[1, i] = self.cy[n - 1]
                xref[2, i] = self.speed[n - 1]
                xref[3, i] = self.cphi[n - 1]
                dref[0, i] = 0.0
        
        self.xref = xref
        return self.xref

    def calc_nearest_index(self):
        '''Helper function for path planner. Returns nearest index'''
        dx = [self.state.x - icx for icx in self.cx[self.index:(self.index + 10)]]
        dy = [self.state.y - icy for icy in self.cy[self.index:(self.index + 10)]]
        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d)) + self.index

        return ind        