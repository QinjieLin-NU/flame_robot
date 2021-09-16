from envs.bipedal_base_env import BipedalBaseEnv
from envs.bipedal_robot import BipedalRobot,BipedalRobot2D
import time
import numpy as np
import gym
class BipedalBulletRLEnvFLAME10Stand(BipedalBaseEnv):
    def __init__(self):
        self.robot = BipedalRobot2D(file_path="urdf/simbicon_urdf/flame9.urdf",delta_t=0.01)
        BipedalBaseEnv.__init__(self, self.robot)

        action_dim = 6 # toruqe of "upperLegBridgeR","lowerLegBridgeR","ankleBridgeR","upperLegBridgeL","lowerLegBridgeL","ankleBridgeL"
        obs_dim = 24
        high = np.ones([action_dim]) * 30.0 #15.0 #5
        high[2], high[5] = 5.0, 5.0
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim]) * 3.14
        self.observation_space = gym.spaces.Box(-high, high)

        # self.joint_angle_limit = np.asarray([1.57,3.14,1.57,1.57,3.14,1.57,1.57]) #center hip, right hip(upper leg), right knee, right ankle, left hip, left knee, left ankle
        self.joint_angle_limit = np.asarray([1.4,1.57,1.57,1.57,1.57,1.57,1.57]) #center hip, right hip(upper leg), right knee, right ankle, left hip, left knee, left ankle
        self.initial_z = None
        self.walk_target_x = 5  # kilometer away
        self.walk_target_y = 0
        self.walk_target_dist_x = self.walk_target_x

        # if next state is in the list, then get positive reward
        # simpler state machine
        # self.next_state_list = {
        #     "leftStand_rightGround": "leftGround_rightStand",
        #     "leftGround_rightGround": "Nothing",
        #     "leftGround_rightStand":"leftStand_rightGround",
        #     "leftStand_rightStand":"Nothing"
        # }
        # self.prev_colision_state = "leftStand_rightGround" #[0,1] # left standing, right ground
        # self.next_reward_colliionStates = self.next_state_list["leftStand_rightGround" ]#[[1,1]]

        # complex
        self.next_state_list={
            "LeftStandFront_RightGroundBack":"LeftGroundFront_RightGroundBack",
            "LeftGroundFront_RightGroundBack":"LeftGroundFront_RightStandBack",
            "LeftGroundFront_RightStandBack":"LeftGroundBack_RightStandFront",
            "LeftGroundBack_RightStandFront":"LeftGroundBack_RightGroundFront",
            "LeftGroundBack_RightGroundFront":"LeftStandBack_RightGroundFront",
            "LeftStandBack_RightGroundFront":"LeftStandFront_RightGroundBack",
            "LeftStandFront_RightStandBack":"LeftGroundFront_RightStandBack",
            "LeftStandBack_RightStandFront":"LeftStandFront_RightGroundBack"
        }
        self.prev_colision_state = "LeftStandFront_RightGroundBack" #[0,1] # left standing, right ground
        self.next_reward_colliionStates = self.next_state_list["LeftStandFront_RightGroundBack" ]#[[1,1]]



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
        actions torque: "upperLegBridgeR","lowerLegBridgeR","ankleBridgeR","upperLegBridgeL","lowerLegBridgeL","ankleBridgeL"
        """
        appeded_actions =  np.array([0.0,a[0],a[1],a[2],a[3],a[4],a[5]])
        self.robot.step(appeded_actions,step_sim=True)
        if(self.real_time):
            time.sleep(self.robot.dt)
        
        state = self.calc_state()
        self.state = np.array(state)
        reward = self.calc_reward(self.state) 

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
        self.prev_colision_state = "LeftStandFront_RightGroundBack" #[0,1] # left standing, right ground
        self.next_reward_colliionStates = self.next_state_list["LeftStandFront_RightGroundBack" ]#[[1,1]]
        # self.prev_colision_state = "leftStand_rightGround" #[0,1] # left standing, right ground
        # self.next_reward_colliionStates = self.next_state_list["leftStand_rightGround" ]#[[1,1]]

        return state
    
    electricity_cost = -2.0	 # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
    stall_torque_cost = -0.1  # cost for running electric current through a motor even at zero rotational speed, small
    foot_collision_cost = -1.0	# touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
    foot_ground_object_names = set(["floor"])  # to distinguish ground and other objects
    joints_at_limit_cost = -0.1	 # discourage stuck joints

    def state_machine(self,left_foot_ground,right_foot_ground,left_footx,right_footx):
        """
        this function return reqrd if robot follow the state machine tranferation
        1 mean ground when collision with floor, 0 means standing wihout collision
        TODO: add right befoe left
        """
        reward = 0.0
        current_collision_state = self.get_collision_state(left_foot_ground,right_foot_ground,left_footx,right_footx)
        # punish double (swing)
        if(current_collision_state == "LeftStandFront_RightStandBack" or current_collision_state == "LeftStandBack_RightStandFront"):
            return -0.1
        #set reward here
        if(current_collision_state == self.next_reward_colliionStates):
            self.next_reward_colliionStates = self.next_state_list[current_collision_state]
            return 0.1

        #record the prvious state
        self.prev_colision_state = current_collision_state
        # self.next_reward_colliionStates = self.next_state_list[current_collision_state]
        # print("current state:",current_collision_state,"next: ",self.next_reward_colliionStates)

        return reward

    def get_collision_state(self,left_foot_ground,right_foot_ground,left_footx,right_footx):
        current_collision_state =  "Left%s%s_Right%s%s"%("Ground" if left_foot_ground else "Stand", "Front" if (left_footx > right_footx) else "Back", "Ground" if right_foot_ground else "Stand", "Front" if (left_footx < right_footx) else "Back")
        # current_collision_state =  "Left%s_Right%s"%("Ground" if left_foot_ground else "Stand", "Ground" if right_foot_ground else "Stand")
        return current_collision_state

    def calc_reward(self,state):

        # calc joint angle and speed 
        j = [self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd]
        joints_at_limit = np.count_nonzero(np.abs(j[0::2]) < self.joint_angle_limit)
        joint_speeds = j[1::2]
        joints_at_limit_cost = 0.03
        joints_cost = joints_at_limit * joints_at_limit_cost#not used
                                
        # return negative when fall
        alive =0   
        done = self.robot.fall_flag
        if(not done):
            alive = 0.01 #0.3
        else:
            return -3

        #double stand reward, implemented inside 
        # double_stand_reward = 0.0
        # right_collision,left_collision = self.robot.right_foot.state,self.robot.left_foot.state
        # right_footx,left_footx = self.robot.right_foot.xyz[0],self.robot.left_foot.xyz[0]

        #stand state machine reward
        # state_weight = 0.0 # 0.2 2.0 0.8
        # statemachine_reward = self.state_machine(left_collision,right_collision,left_footx,right_footx) * state_weight

        #joint speed reward
        joints_move_reward = 0.2 # 0.2
        speed_threshold = 0.001  #0.2
        speed_reward = (np.mean(np.absolute(np.asarray(joint_speeds))) - speed_threshold )* joints_move_reward
        speed_reward = 0.0 if speed_reward > 0 else speed_reward

        #make sure foot is parallel to ground
        foots_rpy = self.robot.right_foot.rpy + self.robot.left_foot.rpy
        foot_weight = 2.0 #0.02 remove this reward
        foot_threshold  = 0.2
        parallelfoot_reward = (foot_threshold - np.mean(np.absolute(np.asarray(foots_rpy)))  )
        parallelfoot_reward = -foot_weight if parallelfoot_reward < 0 else foot_weight
        # print(parallelfoot_reward)

        #make sure knee>0
        knee_angles = np.asarray([j[0::2][2],j[0::2][5]])
        knee_normal = np.count_nonzero( knee_angles > -0.001 )
        knee_weight = 0.02
        knee_reward = 0.0 if knee_normal < 2 else knee_weight

        walk_progress_x = self.last_walk_target_dist_x - self.walk_target_dist_x
        alive_rate = 0.1
        absolute_deltax = np.abs(walk_progress_x)
        walk_threshold = 0.002
        walk_progress_cost = 0.0
        if(absolute_deltax < walk_threshold):
            alive_rate = 1.0
            walk_progress_cost = 1.0
            # speed_reward = 0.0
        # walk_weight = 10.0#10
        # walk_progress_cost = walk_progress_x * walk_weight
        alive_bonus =  alive*self.robot.dt*alive_rate
        

        self.rewards=[walk_progress_cost,alive_bonus,speed_reward,knee_reward,parallelfoot_reward]

        return  sum(self.rewards)
