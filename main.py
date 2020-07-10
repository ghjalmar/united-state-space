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
    if temp_theta > np.pi:
        temp_theta = temp_theta - 2*np.pi
    elif temp_theta < -np.pi:
        temp_theta = -(temp_theta - 2 * np.pi)
    return temp_theta

# Setting up for simulation
dt = 0.1

# Setting up the reference object
n = 4
ref_dict = {1:'constant', 2:'sinus', 3:'linear', 4:'exp'}
K_dict = {1:8, 2:9.5, 3:11, 4:10}
K = K_dict[n]
ref = ref_dict[n]
ref = trackerClass.reference(dt, ref)

# Setting up the tracking object
obj = trackerClass.tracker(ref.dt, 0, 0, 0, 0)
#obj.calc_gain()

# Initial input (reference)
r0 = np.array([[ref.a[0]], [0]])
r_i = r0

# Arrays to store the location of the object that is tracking the reference
x_pos = np.zeros((2, len(ref.x_t)))
d_vec = np.zeros((1, len(ref.x_t)))
theta_vec = np.zeros((2,len(ref.x_t)))
# Feedback loop
for i in range(1, len(ref.x_t)):
    x_pos[0, i] = obj.state_space[0] # x
    x_pos[1, i] = obj.state_space[1] # y
    d, delta_theta = calc_theta(ref.x_t[i-1], ref.y_t[i-1], x_pos[:,i-1])
    temp_theta = turn_angle(delta_theta, obj.state_space[3])
    d_vec[0, i] = d
    theta_vec[0, i] = temp_theta
    theta_vec[1, i] = obj.state_space[3]
    r_i[0] = ref.a[i-1]*K
    r_i[1] = temp_theta
    obj.update_pos(r_i)

print(f'temp_theta[i:j]: {theta_vec[0,0:20]}')
p.plot(ref.x_t, ref.y_t, color='tab:blue')
p.plot(x_pos[0,:], x_pos[1,:], color='tab:red')
p.plot(ref.x_t, theta_vec[1,:], color='tab:green')
p.legend(['reference', 'object', 'heading of object'])
p.show()
p.figure()
p.plot(ref.x_t, theta_vec[0,:], color='tab:blue')
p.plot(ref.x_t[1:], d_vec[0, 1:], color='tab:red')
p.legend(['dTheta_vec', 'd vec'])
p.show()

#p.figure()
#p.plot(ref.v)
#p.legend('ref v')
#p.show()