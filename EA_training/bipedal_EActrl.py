import time
# from calTorque import EA_weights_Controller
import sys
from pybullet_EA_env import PybulletEnv
from calTorque import EA_weights_Controller
import pybullet
import pybullet_data
import time
import numpy as np

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
        multiplier = 1.0

        distTraveled = self.robot.get_dist_traveled()
        energy_remaining = 0.0
        energy_per_meter = 0.0

        if distTraveled >= self.max_distance:
            energy_remaining = self.max_torque - self.accum_torque
            energy_per_meter = self.accum_torque / self.max_distance
            distTraveled = self.max_distance + (energy_remaining / energy_per_meter)

        fitness = distTraveled * multiplier
        return fitness

    # def if_fall(self,fall_flag):
    #     if fall_flag == True:
    #         self.fitness()
    #         break
    #     if fall_flag == False:
    #         return 0

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
        i = 0
        traj_id = 0

        time.sleep(2.0)

        fitness = 0
        self.robot.reset(disable_gui=False, disable_velControl=True, add_debug=False)
        select_traj = self.robot.step_down_init()
        # self.plane = self.p.loadURDF("plane.urdf")
        pattern_count = 0
        switch_count = 0
        # while traj_id<1500:
        #     self.robot.step_down(select_traj,traj_id)
        #     traj_id += 1
        #     time.sleep(self.dt)
        #     collision = self.robot.has_contact(self.p,linkA=self.robot.left_foot.link_id)
        #     if collision[0]==1:
        #         break

        while (i < 100000):

            accum_time = self.robot.dt * i
            if accum_time > 1:
               self.check_flag(pattern_count,switch_count,accum_time)
            if self.fall_flag:
                # self.fitness()
                break
            if not self.fall_flag:
                print("round:",i)
                right_collision = self.robot.has_contact(
                    self.p, linkA=self.robot.right_foot.link_id)
                left_collision = self.robot.has_contact(
                    self.p, linkA=self.robot.left_foot.link_id)
                current_pattern = [left_collision[0], right_collision[0]]

                # right_collision, left_collision = self.robot.right_foot.state, self.robot.right_foot.state
                # left_footx = self.robot.left_foot_traveled()
                # right_footx = self.robot.right_foot_traveled()
                left_foot_dist = self.robot.left_foot_traveled()
                right_foot_dist = self.robot.right_foot_traveled()
                fitness = fitness + self.state_machine(left_collision[0], right_collision[0], left_foot_dist,right_foot_dist)
                print("current fitness:",fitness)

                if left_foot_dist >=right_foot_dist:
                    current_switch_flag = b'left_front'
                else:
                    current_switch_flag = b'right_front'

                if current_pattern != self.robot.collision_pattern and current_pattern != [0,0]:
                    self.robot.collision_pattern=current_pattern
                    pattern_count = 0
                else:
                    pattern_count += 1 # if keep one foot on the ground, stop and punish
                if current_switch_flag != self.robot.switch_flag:
                    self.robot.switch_flag = current_switch_flag
                    switch_count = 0
                else:
                    switch_count += 1 # if keep right or left lef front, stop and punish


                # calculate torques and apply torques to robots
                torques = self.controller.update()
                print("torque:",torques)

                self.robot.step(torques, step_sim=False)
                # robot.step(step_sim=False)

                # step simulation id needed and update state of robot
                self.robot.p.stepSimulation()

                time.sleep(self.robot.dt)
                self.robot.update_state()

                # EA
                self.accum_torque = self.Accum_Torques(torques)


                i += 1


        fitness = fitness + self.fitness() + self.punish
        return fitness
