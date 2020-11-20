from envs.bullet_base_env import BulletBaseEnv
# from pybulletgym.envs.roboschool.scenes import StadiumScene
import pybullet
import numpy as np
import gym
import time 
class BipedalBaseEnv(BulletBaseEnv):
    """
    This class will implement step, reset function of gym envs
    TODO: implement alive bonus function
    """
    def __init__(self, robot, render=False):
        BulletBaseEnv.__init__(self, robot, render)

        # set action and obs space
        action_dim = 7
        obs_dim = 16
        high = np.ones([action_dim])
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim])
        self.observation_space = gym.spaces.Box(-high, high)
        self.camera_x = 0
        self.initial_z =None
        self.potential =0 
        self.joint_angle_limit = np.asarray([3.14,3.14,3.14,3.14,3.14,3.14,3.14])

        #goal: robot walking goal
        self.walk_target_x = 10
        self.walk_target_y = 0

    electricity_cost = -2.0	 # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
    stall_torque_cost = -0.1  # cost for running electric current through a motor even at zero rotational speed, small
    foot_collision_cost = -1.0	# touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
    foot_ground_object_names = set(["floor"])  # to distinguish ground and other objects
    joints_at_limit_cost = -0.1	 # discourage stuck joints

    def step(self, a):
        """
        input: a is the applied torque of the robot
            real_time: if True, sleep every step, else: step very fast without time clock
        step action a and return state
        """
        self.robot.step(a,step_sim=True)
        if(self.real_time):
            time.sleep(0.05)
        self.last_state = np.asarray(self.state)
        self.state=[
            self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd,\
                                        self.robot.left_foot.state,self.robot.right_foot.state]
        state = np.asarray(self.state)
        
        # self.rewards = np.asarray([0,0,0])
        # done= self.robot.fall_flag
        
        alive = float(self.alive_bonus())   
        done = alive < 0
        if(not done):
            done = self.robot.fall_flag

        potential_old = self.potential
        [walk_target_theta,walk_target_dist] = self.calc_distance()
        self.potential = self.calc_potential()
        progress = float(self.potential - potential_old)

        feet_collision_cost = 0.0
        # for i, f in enumerate(self.robot.feet):  # TODO: Maybe calculating feet contacts could be done within the robot code
        #     contact_ids = set((x[2], x[4]) for x in f.contact_list())
        #     # print("CONTACT OF '%d' WITH %d" % (contact_ids, ",".join(contact_names)) )
        #     if self.ground_ids & contact_ids:
        #         # see Issue 63: https://github.com/openai/roboschool/issues/63
        #         # feet_collision_cost += self.foot_collision_cost
        #         self.robot.feet_contact[i] = 1.0
        #     else:
        #         self.robot.feet_contact[i] = 0.0

        self.joint_speeds = [self.robot.center_hip.qd,self.robot.right_hip.qd,self.robot.right_knee.qd,self.robot.right_ankleY.qd,\
            self.robot.left_hip.qd,self.robot.left_knee.qd,self.robot.left_ankleY.qd]
        electricity_cost = self.electricity_cost * float(np.abs(a*self.joint_speeds).mean())  # let's assume we have DC motor with controller, and reverse current braking
        electricity_cost += self.stall_torque_cost * float(np.square(a).mean())

        self.joint_angles =[self.robot.center_hip.q,self.robot.right_hip.q,self.robot.right_knee.q,self.robot.right_ankleY.q,\
            self.robot.left_hip.q,self.robot.left_knee.q,self.robot.left_ankleY.q]
        self.joints_at_limit = np.count_nonzero(np.abs(np.asarray(self.joint_angles)) > self.joint_angle_limit)
        joints_at_limit_cost = float(self.joints_at_limit_cost * self.joints_at_limit)
        # debugmode = 0
        # if debugmode:
        #     print("alive=")
        #     print(alive)
        #     print("progress")
        #     print(progress)
        #     print("electricity_cost")
        #     print(electricity_cost)
        #     print("joints_at_limit_cost")
        #     print(joints_at_limit_cost)
        #     print("feet_collision_cost")
        #     print(feet_collision_cost)

        self.rewards = [
            alive,
            progress,
            electricity_cost,
            joints_at_limit_cost,
            feet_collision_cost
        ]
        # if debugmode:
        #     print("rewards=")
        #     print(self.rewards)
        #     print("sum rewards")
        #     print(sum(self.rewards))
        # self.HUD(state, a, done)
        # self.reward += sum(self.rewards)

        return state, sum(self.rewards), bool(done), {}


    def reset(self):
        """
        reset robot to initial pose 
        """
        self.robot.reset_sim(disable_velControl=True,disable_gui=(not self.isRender),add_debug=False)
        self.robot.update_state()
        self.state=[
            self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_hip.q,self.robot.right_hip.qd, \
                    self.robot.right_knee.q,self.robot.right_knee.qd,\
                        self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                            self.robot.left_hip.q,self.robot.left_hip.qd,\
                                self.robot.left_knee.q,self.robot.left_knee.qd,\
                                    self.robot.left_ankleY.q,self.robot.left_ankleY.qd,\
                                        self.robot.left_foot.state,self.robot.right_foot.state]
        state=np.asarray(self.state)
        return state

    def close(self):
        self.robot.p.disconnect()


    def calc_distance(self):
        """
        return: distance from robot current position to goal
        """
        self.body_xyz = np.asarray(self.robot.torso.torso_pos)
        self.body_rpy = np.asarray(self.robot.torso.torso_ori)
        r,p,yaw = self.body_rpy[0],self.body_rpy[1],self.body_rpy[2]
        z = self.body_xyz[2]
        if self.initial_z is None:
            self.initial_z = z

        self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
                                            self.walk_target_x - self.body_xyz[0])
        self.walk_target_dist = np.linalg.norm(
            [self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])

        return [self.walk_target_theta,self.walk_target_dist]

    def calc_state(self):
        """
        TODO: implement, torso_lin Vel can be changed into angle vel
        """
        self.body_xyz = np.asarray(self.robot.torso.torso_pos)
        self.body_rpy = np.asarray(self.robot.torso.torso_ori)
        r,p,yaw = self.body_rpy[0],self.body_rpy[1],self.body_rpy[2]
        z = self.body_xyz[2]
        if self.initial_z is None:
            self.initial_z = z

        self.walk_target_theta = np.arctan2(self.walk_target_y - self.body_xyz[1],
                                            self.walk_target_x - self.body_xyz[0])
        self.walk_target_dist = np.linalg.norm(
            [self.walk_target_y - self.body_xyz[1], self.walk_target_x - self.body_xyz[0]])

        angle_to_target = self.walk_target_theta - yaw
        rot_speed = np.array(
            [[np.cos(-yaw), -np.sin(-yaw), 0],
             [np.sin(-yaw), np.cos(-yaw), 0],
             [		0,			 0, 1]]
        )

        vx, vy, vz = np.dot(rot_speed, self.robot.torso_linVel)  # rotate speed back to body point of view
        more = np.array([z-self.initial_z,
                          np.sin(angle_to_target), np.cos(angle_to_target),
                          0.3 * vx, 0.3 * vy, 0.3 * vz,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
                          r, p], dtype=np.float32)
        self.feet_contact = np.asarray([self.robot.left_foot.state,self.robot.right_foot.state])
        return np.clip(np.concatenate([more] + self.feet_contact), -5, +5)


    def calc_potential(self):
        """
        claculate potential of robot moving
        """
        # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
        # all rewards have rew/frame units and close to 1.0
        try:
            return - self.walk_target_dist / self.robot.dt
        except AttributeError:
            return - self.walk_target_dist

    def alive_bonus(self,):
        """
        Failure mode: robot doesn't bend knees, tries to walk using hips.
        return positive if robot walk withput bending knee, return negative if beding knee 
        TODO This function need to re implement
        """
        # knees = np.array([j.current_relative_position() for j in [self.jdict["jointLowerLegL"], self.jdict["jointLowerLegR"]]], dtype=np.float32).flatten()
        # knees_at_limit = np.count_nonzero(np.abs(knees[0::2]) > 0.99)
        knees_angle = np.asarray([self.robot.right_knee.q, self.robot.left_knee.q,])
        knees_at_limit = np.count_nonzero(np.abs(knees_angle) > 0.99)
        if(knees_at_limit==2):
            return 1
        else:
            return 2

