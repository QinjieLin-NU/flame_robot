from envs.bipedal_base_env import BipedalBaseEnv
from envs.bipedal_robot import BipedalRobot,BipedalRobot2D
import time
import numpy as np
import gym
class BipedalBulletRLEnvA7D2(BipedalBaseEnv):
    def __init__(self):
        self.robot = BipedalRobot2D(file_path="urdf/simbicon_urdf/flame_2d.urdf")
        BipedalBaseEnv.__init__(self, self.robot)

        action_dim = 7 # toruqe of "upperLegBridgeR","lowerLegBridgeR","ankleBridgeR","upperLegBridgeL","lowerLegBridgeL","ankleBridgeL"
        obs_dim = 24
        high = np.ones([action_dim]) * 5
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim]) * 3.14
        self.observation_space = gym.spaces.Box(-high, high)

        self.joint_angle_limit = np.asarray([1.57,3.14,1.57,1.57,3.14,1.57,1.57])
        self.initial_z = None
        self.walk_target_x = 5  # kilometer away
        self.walk_target_y = 0
        self.walk_target_dist_x = self.walk_target_x


    def calc_state(self):
        """
        return state: [
            status of bodying moving forward
            joint state and speed of thigh_joint leg_joint foot_joint thigh_left_joint leg_left_joint foot_left_joint
            foot contact state
        ]
        """
        # calc joint angle and speed 
        j = [self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd]

        #calc foot contact state
        feet_contact = [self.robot.right_foot.state,self.robot.left_foot.state]
        if(self.robot.right_foot.state == 0 and self.robot.left_foot.state ==0):
            #fake foot here 
            feet_contact=[1,0]

        self.joint_speeds = j[1::2]
        self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > self.joint_angle_limit)

        #calc state aboud body moving forward
        body_pos = self.robot.torso.torso_pos
        body_rpy = self.robot.torso.torso_ori
        body_speed = np.asarray(self.robot.links_Vxyz[0:3]) 
        links_xyz = np.array(self.robot.links_xyz).flatten()
        center_xyz = (
            links_xyz[0::3].mean(), links_xyz[1::3].mean(), body_pos[2])  # torso z is more informative than mean z

        z = center_xyz[2]
        if self.initial_z is None:
            self.initial_z = z
        r, p, yaw = body_rpy[0],body_rpy[1],body_rpy[2]
        self.walk_target_theta = np.arctan2(self.walk_target_y - center_xyz[1],
                                            self.walk_target_x - center_xyz[0])
        self.walk_target_dist = np.linalg.norm(
            [self.walk_target_y - center_xyz[1], self.walk_target_x - center_xyz[0]])
        self.last_walk_target_dist_x = self.walk_target_dist_x
        self.walk_target_dist_x = self.walk_target_x - center_xyz[0]
        angle_to_target = self.walk_target_theta - yaw

        rot_speed = np.array(
            [[np.cos(-yaw), -np.sin(-yaw), 0],
             [np.sin(-yaw), np.cos(-yaw), 0],
             [		0,			 0, 1]]
        )
        vx, vy, vz = np.dot(rot_speed, body_speed)  # rotate speed back to body point of view

        more = np.array([z-self.initial_z,
                          np.sin(angle_to_target), np.cos(angle_to_target),
                          0.3 * vx, 0.3 * vy, 0.3 * vz,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
                          r, p], dtype=np.float32)
        return np.clip(np.concatenate([more] + [j] + [feet_contact]), -5, +5)
    
    def step(self,a):
        """
        the input a should contain 7 values
        """
        appeded_actions =  np.array(a)
        self.robot.step(appeded_actions,step_sim=True)
        if(self.real_time):
            time.sleep(self.robot.dt)
        
        state = self.calc_state()
        self.state = np.array(state)
        reward = self.calc_reward(self.state,a) 

        done = self.robot.fall_flag
        info = {}

        return state,reward,done,info
    
    
    def reset(self):
        """
        reset robot to initial pose 
        """
        self.robot.reset_sim(disable_velControl=True,disable_gui=(not self.isRender),add_debug=False)
        self.robot.update_state()
        state=self.calc_state()
        self.state = np.array(state)
        self.potential = 0
        self.walk_target_dist_x = self.walk_target_x 
        return state
    
    electricity_cost = -2.0	 # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
    stall_torque_cost = -0.1  # cost for running electric current through a motor even at zero rotational speed, small
    foot_collision_cost = -1.0	# touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
    foot_ground_object_names = set(["floor"])  # to distinguish ground and other objects
    joints_at_limit_cost = -0.1	 # discourage stuck joints    

    def calc_reward(self,state,a):

        # calc joint angle and speed 
        j = [self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd]
        joints_at_limit = np.count_nonzero(np.abs(j[0::2]) < self.joint_angle_limit)
        joints_at_limit_cost = 0.03
        joints_cost = joints_at_limit * joints_at_limit_cost

                                
        # return negative when fall
        alive =0   
        done = self.robot.fall_flag
        if(not done):
            alive = 0.3
        else:
            return -3

        walk_progress_x = self.last_walk_target_dist_x - self.walk_target_dist_x
        alive_rate = 1
        if(walk_progress_x<0.0):
            alive_rate = 0.1
        walk_progress_cost = walk_progress_x * 10
        alive_bonus =  alive*self.robot.dt*alive_rate
        

        self.rewards=[walk_progress_cost,alive_bonus]

        return  sum(self.rewards)
