import matplotlib.pyplot as p
import numpy as np

x = np.linspace(0, 2*np.pi)
y = [10*np.sin(val) for val in x]
y2 = [10*np.cos(val) for val in x]
fig, ax = p.subplots()
ax.plot(x, y)

ax1 = ax.twinx()
ax1.plot(x, y2)

p.show()
