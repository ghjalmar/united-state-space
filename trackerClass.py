import numpy as np
import control
import matplotlib.pyplot as p

class tracker:
    # Object that tracks the reference.
    def __init__(self, dt, x=0, y=0, v=1, theta=0):
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
        self.state_space[3] = (self.state_space[3] + reference[1])
        if self.state_space[3] >= 0:
            self.state_space[3] = self.state_space[3]%(2*np.pi)
        else:
            self.state_space[3] = self.state_space[3]%(-2*np.pi)
        # Updates of observed states y. dx and dy are in local frame - need R to rotate to world frame
        self.dx_dy = np.matmul(R, \
            np.array([np.cos(reference[1]) * self.state_space[2] * (self.dt**2),
            np.sin(reference[1]) * self.state_space[2] * (self.dt**2)]))
        self.dx_dy = self.dx_dy.astype(np.float64)
        self.state_space[0] = self.state_space[0] + self.dx_dy[0]
        self.state_space[1] = self.state_space[1] + self.dx_dy[1]

    #def calc_gain(self):
    #    # Calculate new gain.
    #    self.A = np.identity(2)
    #    self.B = np.identity(2)
    #    self.Q = np.identity(2)
    #    self.R = np.identity(2)
    #    self.K, _, _ = control.lqr(self.A, self.B, self.Q, self.R)

class reference:
    def __init__(self, dt, type='constant', plot=0):
        self.dt = dt
        if type == 'constant':
            self.init_constant()
        elif type == 'sinus':
            self.init_sinus()
        elif type == 'linear':
            self.init_linear()
        elif type == 'exp':
            self.init_exp()

        if plot:
            p.plot(self.x_t, self.y_t)
            p.show()

    def __calc_velocity__(self):
        d = np.sqrt(np.power(np.diff(self.x_t), 2) +
                    np.power(np.diff(self.y_t), 2))
        v = d/self.dt
        ret_v = np.array(0)
        ret_v = np.append(ret_v, v)
        a = np.diff(ret_v)/self.dt
        ret_a = np.array(0)
        ret_a = np.append(ret_a, a)
        return ret_v, ret_a

    def init_constant(self):
        # Constant
        x0 = 1
        xn = 150
        n = 300
        self.x_t = np.linspace(x0, xn, n)
        self.y_t = np.array([2 for val in self.x_t])
        self.v, self.a = self.__calc_velocity__()

    def init_sinus(self):
        # Sinus
        x0 = 1
        xn = 150
        n = 3000
        self.x_t = np.linspace(x0, xn, n)
        self.y_t = np.array([0.2*np.sin(0.4 * val) for val in self.x_t])
        self.v, self.a = self.__calc_velocity__()

    def init_linear(self):
        # Linear
        x0 = 1
        xn = 10
        n = 100
        self.x_t = np.linspace(x0, xn, n)
        self.y_t = np.array([2*val for val in self.x_t])
        self.v, self.a = self.__calc_velocity__()

    def init_exp(self):
        # Linear
        x0 = 0
        xn = 10
        n = 100
        self.x_t = np.linspace(x0, xn, n)
        self.y_t = np.array([np.exp(val) for val in self.x_t])
        self.v, self.a = self.__calc_velocity__()