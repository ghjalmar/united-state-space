import matplotlib.pyplot as p
import numpy as np
import trackerClass
import time

# Function declarations
def calc_theta(x_t, y_t, x_pos):
    # Returns the distance to the reference and the angle shift needed to point directly at it, \
    # assuming that the tracker object has 0 heading in world frame.
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
    # Adjusting for the assumption of 0 heading in world frame
    temp_theta = delta_theta - ref_angle
    # if temp_theta < 0:
    #    temp_theta = temp_theta + 2*np.pi
    #temp_theta = temp_theta % (2 * np.pi)
    if temp_theta > np.pi/2:
        temp_theta = temp_theta - 2*np.pi
    return temp_theta

# Setting up for simulation
dt = 0.1

# Setting up the reference object
ref = 'sinus'
ref = trackerClass.reference(dt, ref)

# Setting up the tracking object
obj = trackerClass.tracker(ref.dt, 0, 0, 1, 0)
obj.calc_gain()

# Initial input (reference)
r0 = np.array([[ref.a[0]], [0]])
r_i = r0

# Arrays to store the location of the object that is tracking the reference
x_pos = np.zeros((2, len(ref.x_t)))
d_vec = np.zeros((1, len(ref.x_t)))
theta_vec = np.zeros((2,len(ref.x_t)))
# Feedback loop
for i in range(1, len(ref.x_t)):
    print(r_i)
    print('State space before update:')
    print(obj.state_space)
    obj.update_pos(r_i)
    print('State space after update:')
    print(obj.state_space)
    x_pos[0, i] = obj.state_space[0] # x
    x_pos[1, i] = obj.state_space[1] # y
    temp_Kx = np.matmul(obj.K, r_i)
    d, delta_theta = calc_theta(ref.x_t[i], ref.y_t[i], x_pos[:,i])
    temp_theta = turn_angle(delta_theta, obj.state_space[3])
    d_vec[0, i] = d
    print(f'd: {d}')
    print(f'temp theta: {temp_theta}')
    theta_vec[0,  i] = temp_theta
    theta_vec[1, i] = obj.state_space[3]
    #r_i = r_i - temp_Kx
    r_i[0] = ref.a[i-1]
    r_i[1] = temp_theta

p.plot(ref.x_t, ref.y_t)
p.plot(x_pos[0,:], x_pos[1,:], color='tab:red')
p.legend(['reference', 'object'])
p.show()
p.figure()
p.plot(theta_vec[0,:], color='tab:blue')
p.plot(d_vec[0, 1:], color='tab:red')
p.plot(theta_vec[1,:], color='tab:green')
p.legend(['theta_vec', 'd_vec', 'heading of object'])
p.show()

#p.figure()
#p.plot(ref.v)
#p.legend('ref v')
#p.show()