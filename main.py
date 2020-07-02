import matplotlib.pyplot as p
import numpy as np
import trackerClass

x = np.linspace(0, 2*np.pi)
y = [10*np.sin(val) for val in x]
y2 = [10*np.cos(val) for val in x]
fig, ax = p.subplots()
ax.plot(x, y)

ax1 = ax.twinx()
ax1.plot(x, y2)

#p.show()

obj = trackerClass.tracker()
obj.calc_gain()

r0 = np.array([[1], [0]])
r_i = r0
for i in range(len(x)):
    print(i)
    obj.update_pos(r_i)
    print(obj.state_space)