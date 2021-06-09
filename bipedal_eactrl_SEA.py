from envs.pybullet_env_SEA import PybulletEnv
import time
from controllers.EA_controller_SEA import EA_weights_Controller
import matplotlib.pyplot as plt
import numpy as np

# from calTorque import cal_Torque

if __name__ == "__main__":
   # robot = PybulletEnv(gravity=-10.0,dt=0.001,file_path="urdf/simbicon_urdf/flame3.urdf")
   robot = PybulletEnv(gravity=-10.0, dt=0.001,
                       file_path="/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame4.urdf")
   robot.reset(disable_velControl=True, add_debug=False)
   controller = EA_weights_Controller(robot)
   # allData = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
   allData = []
   i = 0
   time.sleep(2.0)
   while (i < 1000):
       # calculate torques and apply torques to robots
       torques = controller.update()
       # torques_arr = np.array(torques)
       # allData = np.append(allData, torques_arr, axis=1)
       allData.append(torques)
       # print("torques",torques)

       robot.step(torques, step_sim=False)
       # robot.step(step_sim=False)

       # step simulation id needed and update state of robot
       robot.p.stepSimulation()
       time.sleep(robot.dt)
       robot.update_state()

       i += 1
       print("left:", robot.left_foot.state, "front:", robot.left_foot.front_state, "back:",
             robot.left_foot.back_state)
       print("right:", robot.right_foot.state, "front:", robot.right_foot.front_state, "back:",
             robot.right_foot.back_state)
       # if (i%5==0):
       #     robot.left_foot.state = 1
       #     robot.right_foot.state = 0
       # if (i%10==0):
       #     robot.left_foot.state = 0
       #     robot.right_foot.state = 1
       # if(robot.left_foot.state):
       #     print("left:",robot.left_foot.state,"front:",robot.left_foot.front_state,"back:",robot.left_foot.back_state)
       # if(robot.right_foot.state):
       #     print("right:",robot.right_foot.state,"front:",robot.right_foot.front_state,"back:",robot.right_foot.back_state)
   r1 = [x[1] for x in allData]
   r2 = [x[2] for x in allData]
   r4 = [x[4] for x in allData]
   r5 = [x[5] for x in allData]

   l6 = [x[6] for x in allData]
   l7 = [x[7] for x in allData]
   l9 = [x[9] for x in allData]
   l10 = [x[10] for x in allData]

   xpoint = range(1000)

   plt.subplot(2, 1, 1)
   plt.plot(xpoint, r1, label = 'right hipy')
   plt.plot(xpoint, r2, label='right knee')
   plt.plot(xpoint, r4, label='right hipy mot')
   plt.plot(xpoint, r5, label='right knee mot')
   plt.legend()

   plt.subplot(2,1,2)
   plt.plot(xpoint, l6, label='left hipy')
   plt.plot(xpoint, l7, label='left knee')
   plt.plot(xpoint, l9, label='left hipy mot')
   plt.plot(xpoint, l10, label='left knee mot')
   plt.legend()
   plt.show()
   # print("center hip q and qd:", robot.center_hip.q, robot.center_hip.qd)
   # print("right hip q and qd:", robot.right_hip.q, robot.right_hip.qd)
   # print("right knee q and qd:", robot.right_knee.q, robot.right_knee.qd)
   # print("right ankleY q and qd:", robot.right_ankleY.q, robot.right_ankleY.qd)
   # print("left hip q and qd:", robot.left_hip.q, robot.left_hip.qd)
   # print("left knee q and qd:", robot.left_knee.q, robot.left_knee.qd)
   # print("left ankleY q and qd:", robot.left_ankleY.q, robot.left_ankleY.qd)
   # print("left foot state:", robot.left_foot.state)
   # print("right foot state:", robot.right_foot.state)

   time.sleep(10)