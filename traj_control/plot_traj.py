import numpy as np
data  = np.loadtxt("trajectories.txt")

import matplotlib.pyplot as plt
plt.plot(data[:,[0,2]])
plt.show()