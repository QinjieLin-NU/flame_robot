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
        self.controller = EA_weights_Controller(self.robot,self.weights)
        # self.max_dist = 0.001
        self.max_torque = 50000000 #checked!
        self.accum_torque = 0.001

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

    def fitness(self):
        multiplier = 1.0

        distTraveled = self.robot.get_dist_traveled()
        energy_remaining = 0.0
        energy_per_meter = 0.0

        if distTraveled > self.robot.max_distance:
            energy_remaining = self.max_torque - self.accum_torque
            energy_per_meter = self.accum_torque / self.robot.max_distance
            distTraveled = self.robot.max_distance + (energy_remaining / energy_per_meter)

        return distTraveled * multiplier

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

    def check_flag(self,pattern_count):
        # check if we need to stop
        right_foot_collision, right_foot_collision_front, right_foot_collision_back = self.robot.has_contact(self.p, linkA=self.robot.right_foot.link_id)
        left_foot_collision,left_foot_collision_front, left_foot_collision_back = self.robot.has_contact(self.p, linkA=self.robot.left_foot.link_id)
        if pattern_count > 3000:
            self.fall_flag = True
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
        while traj_id<1500:
            self.robot.step_down(select_traj,traj_id)
            traj_id += 1
            time.sleep(self.dt)
            collision = self.robot.has_contact(self.p,linkA=self.robot.left_foot.link_id)
            if collision[0]==1:
                break
        while (i < 100000):
            self.check_flag(pattern_count)
            if self.fall_flag:
                # self.fitness()
                break
            if not self.fall_flag:
                # return 0

                # calculate torques and apply torques to robots
                torques = self.controller.update()

                self.robot.step(torques, step_sim=False)
                # robot.step(step_sim=False)

                # step simulation id needed and update state of robot
                self.robot.p.stepSimulation()

                time.sleep(self.robot.dt)
                self.robot.update_state()

                # EA
                self.accum_torque = self.Accum_Torques(torques)

                right_collision = self.robot.has_contact(
                    self.p, linkA=self.robot.right_foot.link_id)
                left_collision = self.robot.has_contact(
                    self.p, linkA=self.robot.left_foot.link_id)
                current_pattern = [left_collision[0], right_collision[0]]
                if current_pattern != self.robot.collision_pattern:
                    fitness = fitness + 20
                    self.robot.collision_pattern=current_pattern
                    pattern_count = 0
                else:
                    pattern_count += 1
                i += 1
        dist_traveled = self.robot.get_dist_traveled()
        if dist_traveled > self.robot.max_distance:
            self.robot.max_distance = dist_traveled
        fitness = fitness + self.fitness()
        return fitness
