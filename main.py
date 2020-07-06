import matplotlib.pyplot as p
import numpy as np
import trackerClass
import time

# Constant
#x_t = np.linspace(1, 150, 300)
#y_t = [2 for val in x_t]
#v = 1.75
#dt = 0.05

# Sinus
x_t = np.linspace(0, 150, 300)
y_t = [0.5+np.sin(0.04*val) for val in x_t]
v = 0.06
dt = 0.2
#y2_t = [10*np.cos(val) for val in x_t]

# Linear
#x_t = np.linspace(1, 10, 10)
#y_t = [2*val for val in x_t]

p.plot(x_t, y_t)
#p.plot(x_t, y2_t)

#p.show()

# Function declarations
def calc_theta(x_t, y_t, x_pos):
    # Returns the distance to the reference and the angle shift needed to point directly at it.
    delta_x = x_t - x_pos[0]
    delta_y = y_t - x_pos[1]
    d = np.sqrt(delta_x ** 2 + delta_y ** 2)
    if delta_x > 0 and delta_y > 0:
        temp_theta = np.arccos(delta_x / d)
    elif delta_x < 0 and delta_y > 0:
        temp_theta = np.pi/2 + np.arccos(delta_y / d)
    elif delta_x < 0 and delta_y < 0:
        temp_theta = np.pi + np.arccos(abs(delta_x) / d)
    elif delta_x > 0 and delta_y < 0:
        temp_theta = np.pi*3/2 + np.arccos(abs(delta_y) / d)
    else:
        temp_theta = np.arccos(delta_x / d)
    return d, temp_theta

def turn_angle(delta_theta, ref_angle):
    temp_theta = delta_theta - ref_angle
    # if temp_theta < 0:
    #    temp_theta = temp_theta + 2*np.pi
    #temp_theta = temp_theta % (2 * np.pi)
    if temp_theta > np.pi/2:
        temp_theta = temp_theta - 2*np.pi
    return temp_theta
# Setting up the object
obj = trackerClass.tracker(0, 0,1, 0, dt)
obj.calc_gain()

# Initial reference
r0 = np.array([[v], [0]])
r_i = r0

# Arrays to store the location of the object that is tracking the reference
x_pos = np.zeros((2, len(x_t)))
d_vec = np.zeros((1, len(x_t)))
theta_vec = np.zeros((1,len(x_t)))
# Feedback loop
for i in range(1, len(x_t)):
    print(r_i)
    print('State space before update:')
    print(obj.state_space)
    obj.update_pos(r_i)
    print('State space after update:')
    print(obj.state_space)
    x_pos[0, i] = obj.state_space[0] # x
    x_pos[1, i] = obj.state_space[1] # y
    temp_Kx = np.matmul(obj.K, r_i)
    d, delta_theta = calc_theta(x_t[i], y_t[i], x_pos[:,i])
    temp_theta = turn_angle(delta_theta, obj.state_space[3])
    d_vec[0, i] = d
    print(f'd: {d}')
    print(f'temp theta: {temp_theta}')
    theta_vec[0,  i] = temp_theta
    #r_i = r_i - temp_Kx
    r_i[1] = temp_theta

p.plot(x_pos[0,:], x_pos[1,:], color='tab:red')
p.legend(['reference', 'object'])
p.show()
p.figure()
p.plot(theta_vec[0,:], color='tab:blue')
p.plot(d_vec[0, 1:], color='tab:red')
p.legend(['theta_vec', 'd_vec'])
p.show()