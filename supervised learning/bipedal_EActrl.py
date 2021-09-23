import time
# from calTorque import EA_weights_Controller
import sys
from pybullet_EA_env import PybulletEnv
from calTorque import EA_weights_Controller
import pybullet
import pybullet_data
import time
import numpy as np
import math
import os

class bipedal_EActrl():
    def __init__(self, weights,robot):
        self.weights = weights
        self.dt = 0.01

        self.p = pybullet

        self.robot = robot

        self.fall_flag = False
        self.punish = 0
        self.controller = EA_weights_Controller(self.robot,self.weights)
        # self.max_dist = 0.001
        self.max_torque = 2000000 #checked! 50000000
        self.accum_torque = 0.001
        self.accum_err = 0
        self.max_distance = 100
        self.next_state_list = {
            "LeftGroundFront_RightStandBack": "LeftGroundBack_RightStandFront",
            "LeftGroundBack_RightStandFront": "LeftGroundBack_RightGroundFront",
            "LeftGroundBack_RightGroundFront": "LeftStandBack_RightGroundFront",
            "LeftStandBack_RightGroundFront": "LeftStandFront_RightGroundBack",
            "LeftStandFront_RightGroundBack": "LeftGroundFront_RightGroundBack",
            "LeftGroundFront_RightGroundBack": "LeftGroundFront_RightStandBack",
        }
        self.prev_colision_state = "LeftGroundFront_RightStandBack"  # [0,1] # left standing, right ground
        self.next_reward_colliionStates = self.next_state_list["LeftGroundFront_RightStandBack"]  # [[1,1]]

    def Accum_Torques(self, torques):
        C1 = abs(torques[0]) * self.dt  # centerHip_torque
        C2 = abs(torques[1]) * self.dt  # rightHip_torque
        C3 = abs(torques[2]) * self.dt  # rightKnee_torque
        C4 = abs(torques[3]) * self.dt  # rightAnkleY_torque
        C5 = abs(torques[4]) * self.dt  # leftHip_torque
        C6 = abs(torques[5]) * self.dt  # leftKnee_torque
        C7 = abs(torques[6]) * self.dt  # leftAnkleY_torque
        self.accum_torque = self.accum_torque + C1 + C2 + C3 + C4 + C5 + C6 + C7
        return self.accum_torque

    def Accum_Err(self, traj_id, traj):
        # err0 = math.pow((self.robot.right_hip.q - traj[0, traj_id]), 2)
        # err1 = math.pow((self.robot.right_hip.q - traj[1, traj_id]), 2)
        # err2 = math.pow((self.robot.right_hip.q - traj[2, traj_id]), 2)
        # err3 = math.pow((self.robot.right_hip.q - traj[3, traj_id]), 2)
        # err4 = math.pow((self.robot.right_hip.q - traj[4, traj_id]), 2)
        # err5 = math.pow((self.robot.right_hip.q - traj[5, traj_id]), 2)
        err0 = abs(self.robot.center_hip.q - traj[0][traj_id])
        err1 = abs(self.robot.left_knee.q - traj[1][traj_id])
        err2 = abs(self.robot.right_knee.q - traj[2][traj_id])
        err3 = abs(self.robot.left_hip.q - traj[3][traj_id])
        err4 = abs(self.robot.right_hip.q - traj[4][traj_id])
        err5 = abs(self.robot.left_ankleY.q - traj[5][traj_id])
        err6 = abs(self.robot.right_ankleY.q - traj[6][traj_id])

        self.accum_err = self.accum_err + (err6 + err5 + err4 + err3 + err2 + err1 + err0)/6
        print("accum_err:",self.accum_err)


    def get_collision_state(self,left_foot_ground,right_foot_ground,left_footx,right_footx):
        current_collision_state =  "Left%s%s_Right%s%s"%("Ground" if left_foot_ground else "Stand", "Front" if (left_footx > right_footx) else "Back", "Ground" if right_foot_ground else "Stand", "Front" if (left_footx < right_footx) else "Back")
        return current_collision_state

    def state_machine(self,left_foot_ground,right_foot_ground,left_footx,right_footx):
        """
        this function return reqrd if robot follow the state machine tranferation
        1 mean ground when collision with floor, 0 means standing wihout collision
        TODO: add right befoe left
        """
        reward = 0.0
        current_collision_state = self.get_collision_state(left_foot_ground,right_foot_ground,left_footx,right_footx)
        #set reward here
        if(current_collision_state == self.next_reward_colliionStates):
            print("switch state to:",current_collision_state)
            self.prev_colision_state = current_collision_state
            self.next_reward_colliionStates = self.next_state_list[current_collision_state]
            return 100.0

        #record the prvious state
        # print("prev state:", self.prev_colision_state)
        # self.prev_colision_state = current_collision_state
        # self.next_reward_colliionStates = self.next_state_list[current_collision_state]
        # print("current state:",current_collision_state,"next: ",self.next_reward_colliionStates)

        return reward

    def fitness(self):
        fitness = self.accum_err
        return fitness

    def has_collision(self,bodyA,linkA):
        """
        o means no contact
        """
        if len(self.p.getContactPoints(bodyA, self.plane, linkIndexA=linkA)) == 0:
            return 0
        else:
            return 1

    def check_flag(self,pattern_count,switch_count,t):
        # check if we need to stop
        # check if the walking pattern is wrong
        right_foot_collision, right_foot_collision_front, right_foot_collision_back = self.robot.has_contact(self.p, linkA=self.robot.right_foot.link_id)
        left_foot_collision,left_foot_collision_front, left_foot_collision_back = self.robot.has_contact(self.p, linkA=self.robot.left_foot.link_id)
        if pattern_count > 1000:
            self.fall_flag = True
            self.punish += -5000
        if switch_count > 1000:
            self.fall_flag = True
            self.punish += -5000
        # check if the energy is consumed
        if self.accum_torque >= self.max_torque:
            self.fall_flag = True

        # check if the walking distance is finished
        distTraveled = self.robot.get_dist_traveled()
        if distTraveled >= self.max_distance:
            self.fall_flag = True

        # check if the speed is too fast
        if t==0:
            vel = 0
        else:
            vel = self.robot.get_vel(t)
            print("velocity:",vel)
        if vel > 2 and t > 1:
            self.fall_flag = True
            self.punish += -2000

        # check if fly
        if (right_foot_collision == 0) and (left_foot_collision == 0):
            self.fall_flag = True
        # check if fall down
        for j in range(self.p.getNumJoints(self.robot.humanoid)):
            if j != 15 and j != 7:
                collision = self.robot.has_contact(self.p, linkA=j)
                if collision[0] == 1:
                    self.fall_flag = True


    def move(self):
        traj_id = 0
        fitness = 0
        self.robot.reset(disable_gui=True, disable_velControl=True, add_debug=False)

        # path1 = os.path.abspath('..')
        # knee_left_traj = np.loadtxt("../generate_trajectory/knee_left.csv")  # 13
        # knee_right_traj = np.loadtxt("../generate_trajectory/knee_right.csv")  # 5
        # hip_left_traj = np.loadtxt("../generate_trajectory/hip_left.csv")  # 12
        # hip_right_traj = np.loadtxt("../generate_trajectory/hip_right.csv")  # 4
        # ankle_left_traj = np.loadtxt("../generate_trajectory/ankle_left.csv")  # 14
        # ankle_right_traj = np.loadtxt("../generate_trajectory/ankle_right.csv")  # 6
        # traj = [knee_left_traj,knee_right_traj,hip_left_traj,hip_right_traj,ankle_left_traj,ankle_right_traj]
        theta_list = np.loadtxt("theta_3d.csv", delimiter=',')
        theta_list = np.array(theta_list)
        center_hip_traj = theta_list[:,0]
        hip_right_traj = theta_list[:,1]
        knee_right_traj = theta_list[:,2]
        ankle_right_traj = theta_list[:,3]
        hip_left_traj = theta_list[:,4]
        knee_left_traj = theta_list[:,5]
        ankle_left_traj = theta_list[:,6]

          
        # knee_left_tau = np.loadtxt("torque.csv")  # 13
        # knee_right_tau = np.loadtxt("../generate_trajectory/knee_right.csv")  # 5
        # hip_left_tau = np.loadtxt("../generate_trajectory/hip_left.csv")  # 12
        # hip_right_tau = np.loadtxt("../generate_trajectory/hip_right.csv")  # 4
        # ankle_left_tau = np.loadtxt("../generate_trajectory/ankle_left.csv")  # 14
        # ankle_right_tau = np.loadtxt("../generate_trajectory/ankle_right.csv")  # 6
        traj = [center_hip_traj,knee_left_traj,knee_right_traj,hip_left_traj,hip_right_traj,ankle_left_traj,ankle_right_traj]
        # self.plane = self.p.loadURDF("plane.urdf")

        while (traj_id < 300):
            print("round:", traj_id)

            # calculate torques and apply torques to robots
            torques = self.controller.update()
            print("torque:", torques)

            self.robot.step(torques, step_sim=False)
            # robot.step(step_sim=False)

            # step simulation id needed and update state of robot
            self.robot.p.stepSimulation()

            time.sleep(self.robot.dt)
            self.robot.update_state()

            # EA
            self.Accum_Err(traj_id, traj)

            right_collision = self.robot.has_contact(
                self.p, linkA=self.robot.right_foot.link_id)
            left_collision = self.robot.has_contact(
                self.p, linkA=self.robot.left_foot.link_id)
            current_pattern = [left_collision[0], right_collision[0]]
            print(current_pattern)

            traj_id += 1
        fitness = self.fitness()
        return fitness
