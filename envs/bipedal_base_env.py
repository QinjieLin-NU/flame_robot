from envs.bullet_base_env import BulletBaseEnv
# from pybulletgym.envs.roboschool.scenes import StadiumScene
import pybullet
import numpy as np
import gym

class BipedalBaseEnv(BulletBaseEnv):
    """
    This class will implement step, reset function of gym envs
    TODO: implement calc state and calc potential
    """
    def __init__(self, robot, render=False):
        """
        implementation of bullet env of GYM interface
        """
        BulletBaseEnv.__init__(self, robot, render)
        action_dim = 7
        obs_dim = 12
        high = np.ones([action_dim])
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim])
        self.observation_space = gym.spaces.Box(-high, high)
        self.camera_x = 0

    def step(self, a):
        """
        input: a is the applied torque of the robot
        step action a and return state
        TODO: cal state, calc rewards
        """
        self.robot.step(a,step_sim=True)
        self.state=[
            self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_knee.q,self.robot.right_knee.qd,\
                    self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                        self.robot.left_knee.q,self.robot.left_knee.qd,\
                            self.robot.left_ankleY.q,self.robot.left_ankleY.qd,\
                                self.robot.left_foot.state,self.robot.right_foot.state]
        state = np.asarray(self.state)
        self.rewards = np.asarray([0,0,0])
        done= self.robot.fall_flag

        # state = self.robot.calc_state()  # also calculates self.joints_at_limit

        # alive = float(self.robot.alive_bonus(state[0] + self.robot.initial_z, self.robot.body_rpy[1]))   
        # # state[0] is body height above ground, body_rpy[1] is pitch
        # done = alive < 0
        # if not np.isfinite(state).all():
        #     print("~INF~", state)
        #     done = True

        # potential_old = self.potential
        # self.potential = self.robot.calc_potential()
        # progress = float(self.potential - potential_old)

        # feet_collision_cost = 0.0
        # for i, f in enumerate(self.robot.feet):  # TODO: Maybe calculating feet contacts could be done within the robot code
        #     contact_ids = set((x[2], x[4]) for x in f.contact_list())
        #     # print("CONTACT OF '%d' WITH %d" % (contact_ids, ",".join(contact_names)) )
        #     if self.ground_ids & contact_ids:
        #         # see Issue 63: https://github.com/openai/roboschool/issues/63
        #         # feet_collision_cost += self.foot_collision_cost
        #         self.robot.feet_contact[i] = 1.0
        #     else:
        #         self.robot.feet_contact[i] = 0.0

        # electricity_cost = self.electricity_cost * float(np.abs(a*self.robot.joint_speeds).mean())  # let's assume we have DC motor with controller, and reverse current braking
        # electricity_cost += self.stall_torque_cost * float(np.square(a).mean())

        # joints_at_limit_cost = float(self.joints_at_limit_cost * self.robot.joints_at_limit)
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

        # self.rewards = [
        #     alive,
        #     progress,
        #     electricity_cost,
        #     joints_at_limit_cost,
        #     feet_collision_cost
        # ]
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
        TODO: return state,reward
        """
        self.robot.reset_sim(disable_velControl=True,disable_gui=(not self.isRender),add_debug=False)
        self.robot.update_state()
        state=[
            self.robot.center_hip.q,self.robot.center_hip.qd,\
                self.robot.right_knee.q,self.robot.right_knee.qd,\
                    self.robot.right_ankleY.q,self.robot.right_ankleY.qd,\
                        self.robot.left_knee.q,self.robot.left_knee.qd,\
                            self.robot.left_ankleY.q,self.robot.left_ankleY.qd,\
                                self.robot.left_foot.state,self.robot.right_foot.state]
        state=np.asarray(state)
        return state

    def close(self):
        self.robot.p.disconnect()


    electricity_cost = -2.0	 # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
    stall_torque_cost = -0.1  # cost for running electric current through a motor even at zero rotational speed, small
    foot_collision_cost = -1.0	# touches another leg, or other objects, that cost makes robot avoid smashing feet into itself
    foot_ground_object_names = set(["floor"])  # to distinguish ground and other objects
    joints_at_limit_cost = -0.1	 # discourage stuck joints


    def calc_state(self):
        """
        TODO: implement
        """
        j = np.array([j.current_relative_position() for j in self.ordered_joints], dtype=np.float32).flatten()
        # even elements [0::2] position, scaled to -1..+1 between limits
        # odd elements  [1::2] angular speed, scaled to show -1..+1
        self.joint_speeds = j[1::2]
        self.joints_at_limit = np.count_nonzero(np.abs(j[0::2]) > 0.99)

        body_pose = self.robot_body.pose()
        parts_xyz = np.array([p.pose().xyz() for p in self.parts.values()]).flatten()
        self.body_xyz = (
            parts_xyz[0::3].mean(), parts_xyz[1::3].mean(), body_pose.xyz()[2])  # torso z is more informative than mean z
        self.body_rpy = body_pose.rpy()
        z = self.body_xyz[2]
        if self.initial_z is None:
            self.initial_z = z
        r, p, yaw = self.body_rpy
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
        vx, vy, vz = np.dot(rot_speed, self.robot_body.speed())  # rotate speed back to body point of view

        more = np.array([z-self.initial_z,
                          np.sin(angle_to_target), np.cos(angle_to_target),
                          0.3 * vx, 0.3 * vy, 0.3 * vz,  # 0.3 is just scaling typical speed into -1..+1, no physical sense here
                          r, p], dtype=np.float32)
        return np.clip(np.concatenate([more] + [j] + [self.feet_contact]), -5, +5)

    def calc_potential(self):
        """
        implement
        """
        # progress in potential field is speed*dt, typical speed is about 2-3 meter per second, this potential will change 2-3 per frame (not per second),
        # all rewards have rew/frame units and close to 1.0
        try:
            debugmode = 0
            if debugmode:
                print("calc_potential: self.walk_target_dist")
                print(self.walk_target_dist)
                print("self.scene.dt")
                print(self.scene.dt)
                print("self.scene.frame_skip")
                print(self.scene.frame_skip)
                print("self.scene.timestep")
                print(self.scene.timestep)
            return - self.walk_target_dist / self.scene.dt
        except AttributeError:
            return - self.walk_target_dist

    def alive_bonus(self, z, pitch):
        # This is debug code to fix unwanted self-collisions:
        #for part in self.parts.values():
        #	contact_names = set(x.name for x in part.contact_list())
        #	if contact_names:
        #		print("CONTACT OF '%s' WITH '%s'" % (part.name, ",".join(contact_names)) )

        #x, y, z = self.head.pose().xyz()
        # Failure mode: robot doesn't bend knees, tries to walk using hips.
        # We fix that by a bit of reward engineering.
        knees = np.array([j.current_relative_position() for j in [self.jdict["jointLowerLegL"], self.jdict["jointLowerLegR"]]], dtype=np.float32).flatten()
        knees_at_limit = np.count_nonzero(np.abs(knees[0::2]) > 0.99)
        return +4-knees_at_limit if z > 1.3 else -1

