import numpy as np
import control

class tracker:
    # Object that tracks the reference.
    def __init__(self, x=0, y=0, v=1, theta=0, dt=1):
        self.dt = dt
        # State space consists of [x, y, v, theta]^T
        self.state_space = np.array([x, y, v, theta])
        self.state_space = self.state_space.astype(np.float64)

    def RotMat(self, theta):
        # Lambda function for rotation matrix
        return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def update_pos(self, reference):
        # Updates the position of the tracker object. Takes in an np.array with control input dV and dTheta.
        R = self.RotMat(self.state_space[3])
        # Updates of inner states X = [v theta]^T
        self.state_space[2] = self.state_space[2] + reference[0]*self.dt
        self.state_space[3] = (self.state_space[3] + reference[1]*self.dt)%(2*np.pi)
        # Updates of observed states y. dx and dy are in local frame - need R to rotate to world frame
        self.dx_dy = np.matmul(R, \
            np.array([np.cos(reference[1]) * self.state_space[2] * self.dt,
            np.sin(reference[1]) * self.state_space[2] * self.dt]))
        self.dx_dy = self.dx_dy.astype(np.float64)
        self.state_space[0] = self.state_space[0] + self.dx_dy[0]
        self.state_space[1] = self.state_space[1] + self.dx_dy[1]

    def calc_gain(self):
        # Calculate new gain.
        self.A = np.identity(2)
        self.B = np.identity(2)
        self.Q = np.identity(2)
        self.R = np.identity(2)
        self.K, _, _ = control.lqr(self.A, self.B, self.Q, self.R)
