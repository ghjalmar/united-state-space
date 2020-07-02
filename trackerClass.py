import numpy as np
import control

class tracker:
    # Object that tracks the reference.
    def __init__(self, x=0, y=0, theta=0, v=1, dt=1):
        self.dt = dt
        # State space consists of [x, y, theta, v]^T
        self.state_space = np.transpose(np.array([x, y, theta, v]))
    def update_pos(self, reference):
        # Updates the position of the tracker object. Takes in an np.array with control input dV and dTheta.
        self.state_space[0] = self.state_space[0] + np.cos(reference[1])*reference[0]*self.dt
        self.state_space[1] = self.state_space[1] + np.sin(reference[1])*reference[0]*self.dt
        self.state_space[2] = self.state_space[2] + reference[0]
        self.state_space[3] = self.state_space[3] + reference[1]

    def calc_gain(self):
        # Calculate new gain.
        self.A = np.identity(2)
        self.B = np.identity(2)
        self.Q = np.identity(2)
        self.R  = np.identity(2)
        self.K, _, _ = control.lqr(self.A, self.B, self.Q, self.R)