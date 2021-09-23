from pybullet_EA_env import PybulletEnv
import time
from calTorque import EA_weights_Controller
from train import read_csv,reshape
import pybullet as p
import numpy as np
# from calTorque import cal_Torque

if __name__ == "__main__":
    weight = read_csv("results_3_3d.csv")
    weight = weight[0,0:-1]
    parent0 = np.reshape(weight,(1,196))
    print(parent0,type(parent0))
    dt = 0.01
    robot = PybulletEnv(gravity=-10.0, dt=0.01,file_path="../urdf/simbicon_urdf/flame8.urdf")
    robot.reset(disable_gui=False, disable_velControl=True, add_debug=False)
    controller = EA_weights_Controller(robot,parent0)
    i = 0
    traj_id = 0
    fitness = 0
    while (traj_id < 2000):
        # calculate torques and apply torques to robots
        print("step:",traj_id)
        torques = controller.update()
        # print("torques",torques)

        robot.step(torques, step_sim=False)
        # robot.step(step_sim=False)

        # step simulation id needed and update state of robot
        robot.p.stepSimulation()
        time.sleep(dt)
        robot.update_state()



        traj_id += 1
        # vel = robot.get_vel(dt*i)
        print(torques)
        # print('====vel=======:',vel,robot.get_dist_traveled(),dt*i)
        # print("left:", robot.left_foot.state, "front:", robot.left_foot.front_state, "back:",
        #       robot.left_foot.back_state)
        # print("right:", robot.right_foot.state, "front:", robot.right_foot.front_state, "back:",
        #       robot.right_foot.back_state)
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

    print("center hip q and qd:", robot.center_hip.q, robot.center_hip.qd)
    print("right hip q and qd:", robot.right_hip.q, robot.right_hip.qd)
    print("right knee q and qd:", robot.right_knee.q, robot.right_knee.qd)
    print("right ankleY q and qd:", robot.right_ankleY.q, robot.right_ankleY.qd)
    print("left hip q and qd:", robot.left_hip.q, robot.left_hip.qd)
    print("left knee q and qd:", robot.left_knee.q, robot.left_knee.qd)
    print("left ankleY q and qd:", robot.left_ankleY.q, robot.left_ankleY.qd)
    print("left foot state:", robot.left_foot.state)
    print("right foot state:", robot.right_foot.state)

    time.sleep(10)