import matplotlib.pyplot as plt
import pickle
import numpy as np

data = None
file_name = "ray_example/log/PPO_my_env_2021-08-23_22-00-43bnev_f58.pkl" #3d 6torque
# file_name = "ray_example/log/PPO_my_env_2021-08-25_23-14-21vuz8e69y.pkl" #2d 6torques
with open(file_name, 'rb') as f:
    data = pickle.load(f)

plt.plot(data)
plt.legend()
plt.show()
