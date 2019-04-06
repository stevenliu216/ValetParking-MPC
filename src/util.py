'''Defines state, control classes'''

class vehicle_state:
    '''Represents the state of the vehicle'''
    def __init__(self, x, y, v, phi):
        self.x = x
        self.y = y
        self.v = v
        self.phi = phi

class control_input:
    '''Represents the control input'''
    def __init__(self, a, delta):
        self.a = a
        self.delta = delta