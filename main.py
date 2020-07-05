import matplotlib.pyplot as p
import numpy as np
import trackerClass
import time

# Consant
x_t = np.linspace(1, 1.5, 200)
y_t = [1 for val in x_t]

# Sinus
#x_t = np.linspace(0, np.pi/4, 100)
#y_t = [10*np.sin(0.01*val) for val in x_t]
#y2_t = [10*np.cos(val) for val in x_t]

# Linear
#x_t = np.linspace(1, 10, 10)
#y_t = [2*val for val in x_t]

p.plot(x_t, y_t)
#p.plot(x_t, y2_t)

#p.show()

# Setting up the object
obj = trackerClass.tracker()
obj.calc_gain()

# Initial reference
r0 = np.array([[0.01], [0]])
r_i = r0

# Arrays to store the location of the object that is tracking the reference
x_pos = np.zeros((2, len(x_t)))
d_vec = np.zeros((1, len(x_t)))
theta_vec = np.zeros((1,len(x_t)))
for i in range(1, len(x_t)):
    print(r_i)
    print('State space:')
    print(obj.state_space)
    obj.update_pos(r_i)
    x_pos[0, i] = obj.state_space[0] # x
    x_pos[1, i] = obj.state_space[1] # y
    temp_Kx = np.matmul(obj.K, r_i)
    d = np.sqrt((x_t[i] - x_pos[0, i])**2 + (x_t[i] - x_pos[1, i])**2)
    d_vec[0, i] = d
    print(f'd: {d}')
    temp_theta = np.arcsin(obj.dx/d) #- obj.state_space[3]
    print(f'temp theta: {temp_theta}')
    theta_vec[0,  i] = temp_theta
    #r_i = r_i - temp_Kx
    r_i[1] = 2 * temp_theta

p.plot(x_pos[0,:], x_pos[1,:], color='tab:red')
p.show()
p.figure()
p.plot(theta_vec[0,:], color='tab:red')
#p.plot(d_vec[0, :])
p.show()