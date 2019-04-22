'''Defines state, control classes'''

from .spline_interpolation import *

class vehicle_state:
    '''Represents the state of the vehicle'''
    def __init__(self, x, y, v, phi):
        self.x = x
        self.y = y
        self.v = v
        self.phi = phi

    def print_state(self):
        print("State: ", self.x, self.y, self.v, self.phi)

class control_input:
    '''Represents the control input'''
    def __init__(self, a, delta):
        self.a = a
        self.delta = delta

def get_test_track(xlist, ylist):
    '''Returns a test track consisting of (x, y, phi, k) for each course tick
        inputs: xlist, ylist are lists of waypoint coordinates
    '''
    ds = 1  # Let ds be the course tick
    sp = Spline2D(xlist, ylist)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, rphi, rk = [], [], [], []
    for index in s:
        ix, iy = sp.calc_position(index)
        rx.append(ix)
        ry.append(iy)
        rphi.append(sp.calc_phi(index))
        rk.append(sp.calc_curvature(index))
    rphi = smooth_yaw(rphi)
    return rx, ry, rphi, rk

def generate_speed_profile(test_track, target_speed):
    '''Simulates an online path planner by return a target speed for each tick'''
    size_of_track = len(test_track[0])
    speed_profile = [target_speed] * size_of_track # List the length of test track
    direction = 1.0 # Let 1 be forward direction

    for i in range(size_of_track-1):
        dx = test_track[0][i+1] - test_track[0][i]
        dy = test_track[1][i+1] - test_track[1][i]
        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi2pi(move_direction - test_track[2][i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0
        
        if i >= (size_of_track-30):
            target_speed = 5.0 / 3.6

        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed
    
    speed_profile[-1] = 0.0

    return speed_profile

def pi2pi(angle):
    '''Helper function makes sure angle is -pi to pi'''
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw