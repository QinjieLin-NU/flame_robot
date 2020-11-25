from envs.bipedal_base_env import BipedalBaseEnv
from envs.bipedal_robot import BipedalRobot
# from pybulletgym.envs.roboschool.scenes import StadiumScene
import time
import numpy as np
import gym


MAX_TORQUE=2e3
dt=0.001
GRAVITY=-9.8
mu = 0.65
kp = np.array([ 0. ,  0.3,  0.2,  0.1,  0.3,  0.2,  0.1])
kd = 0.1*kp
max_iters = 5e4
target_vel = 0.#3.5

class BipedalBulletSimbiconEnv(BipedalBaseEnv):
    def __init__(self):
        self.robot = BipedalRobot()
        BipedalBaseEnv.__init__(self, self.robot)

        # set action and obs space
        action_dim = 7
        obs_dim = 56
        high = np.ones([action_dim])
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim])

        #extra aegus for simbiconn
        self.mu = mu
        self.max_torque = MAX_TORQUE
        self.max_iters = 5e4
        self.g = GRAVITY
        self.kp = np.array([ 0. ,  0.3,  0.2,  0.1,  0.3,  0.2,  0.1])
        self.kd =  0.1*kp
        self.scale=1.
        self.link_names=["body","upperLegBridgeR","lowerLegBridgeR","ankleBridgeR","upperLegBridgeL","lowerLegBridgeL","ankleBridgeL"]

        #init pose for debug line
        init_pos = [0,0,0]
        self.prevPoses={"body":init_pos,"upperLegBridgeR":init_pos,"lowerLegBridgeR":init_pos,"ankleBridgeR":init_pos,"upperLegBridgeL":init_pos,"lowerLegBridgeL":init_pos,"ankleBridgeL":init_pos}
        self.hasPrevPoses = {"body":0,"upperLegBridgeR":0,"lowerLegBridgeR":0,"ankleBridgeR":0,"upperLegBridgeL":0,"lowerLegBridgeL":0,"ankleBridgeL":0}
    
    def get_links(self):
        #update link name id dictionary
        link_name_id_dict={}
        for j in range (self.robot.p.getNumJoints(self.robot.humanoid)):
            info = self.robot.p.getJointInfo(self.robot.humanoid,j)
            link_id = info[0]
            link_name = info[12].decode('UTF-8')
            link_name_id_dict.update({link_name:link_id})
            print(info)
        for link_name in self.link_names:
            print(link_name,link_name_id_dict[link_name])
        return link_name_id_dict
    
    def get_link_joint_state(self):
        """
        get link and joint state
        """
        #update joint state 
        joint_states =[
            self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd]
        
        #update link state
        link_states = []
        for link_name in self.link_names:
            link_id = self.link_name_id_dict[link_name]
            link_state = self.robot.p.getLinkState(self.robot.humanoid, link_id, computeLinkVelocity=1)
            x,y,z = link_state[0][0],link_state[0][1],link_state[0][2]
            xv,yv,zv = link_state[-2][0],link_state[-2][1],link_state[-2][2]
            link_states += [x,y,z,xv,yv,zv]
            # if(link_name =="body"):
                # print("body pos:",x,y,z)
            #draw line for debug
            # self.pos = [x,y,z]
            # if (self.hasPrevPoses[link_name]==1):
            #     self.robot.p.addUserDebugLine(self.prevPoses[link_name],self.pos,[0,0,0.3],1,15)
            # self.hasPrevPoses[link_name] = 1	
            # self.prevPoses[link_name]=self.pos


        #update foot state
        #TODO: delete default foot state
        foot_states = [self.robot.right_foot.state,self.robot.left_foot.state]
        if(self.robot.right_foot.state == 0 and self.robot.left_foot.state ==0):
            # print("fake foot") #fake foot here 
            foot_states=[1,0]

        state = list(joint_states+link_states+foot_states)
        return state
    
    def pd_control(self, action):
        """Performs PD control for target angles (given by action)
        action = actions of [body,right-hip,right-knee,right-ankle,left-gip,left-knee,left-ankle]
        """
        self.joint_idx = [0,self.robot.right_hip.joint_id,self.robot.right_knee.joint_id,self.robot.right_ankleY.joint_id,\
            self.robot.left_hip.joint_id,self.robot.left_knee.joint_id,self.robot.left_ankleY.joint_id] #body index=0 might be wrong, but body will not use and we don't care
        for i,j in enumerate(self.joint_idx[1:]):
            if action[i] is not None and not np.isnan(action[i]):
                torque_i = -self.kp[i]*(self.robot.p.getJointState(self.robot.humanoid, j)[0]-action[i]) - self.kd[i]*self.robot.p.getJointState(self.robot.humanoid, j)[1]
                self.robot.p.setJointMotorControl2(self.robot.humanoid, j, self.robot.p.TORQUE_CONTROL, force=np.clip(self.scale*torque_i, -self.max_torque,self.max_torque))
        self.robot.p.stepSimulation()
    

    def robot_specific_reset(self):
        """
        TODO: reset robot on designed positions
        """
        self.robot.robot_specific_reset()
    
    def step(self,a):
        #preprocess action when zero
        a = np.clip(a,-self.max_torque,self.max_torque)
        pd_control_action =  np.append(0.0,(a[:6]))
        torque_control_action = np.append(0.0,np.nan_to_num(a[6:]))
        # print("action: ",torque_control_action)

        self.pd_control(pd_control_action)
        self.robot.step(torque_control_action,step_sim=True)
        if(self.real_time):
            time.sleep(self.robot.dt)
        
        self.state = self.get_link_joint_state()
        state = np.asarray(self.state)
        reward = 0.0
        done = self.robot.fall_flag
        info = {}

        return state,reward,done,info

    def reset(self):
        base_observation = BipedalBaseEnv.reset(self)
        #add step down:
        # test_visual = self.robot.p.createVisualShape(self.robot.p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 1])
        # test_collision = self.robot.p.createCollisionShape(self.robot.p.GEOM_BOX, halfExtents=[0.2,1,0.05])
        # test_body = self.robot.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
        # baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])
        #get link id dictionary
        self.link_name_id_dict = self.get_links()
        #update state
        self.state = self.get_link_joint_state()
        state = np.asarray(self.state)
        return state
