import numpy as np
import control

class tracker(x = 0, y = 0, theta = 0, v = 1):
    # Object that tracks the reference.
    def __init__(self, x, y, theta, v):
        # State space consists of [x, y, theta, v]^T
        self.state_space = np.transpose(np.array([x, y, theta, v]))
    def update_pos(self, reference):
        # Updates the position of the tracker object. Takes in an np.array with x and y pos of object to track.

    def calc_gain(self):
        # Calculate new gain.
        self.A = np.identity(4)
        self.B = np.array([[cos(self.theta), 0], [sin(self.theta), 0], [0, 1], [1, 0]]) # Needs to be linearized!
        self.Q = np.identity(4)
        self.R  = np.identity(4)
        self.K, _, _ = control.lqr(self.A, self.B, self.Q, self.R)