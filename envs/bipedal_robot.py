from envs.urdf_base_robot import URDFBaseRobot
import numpy as np
import pybullet as p
import gym, gym.spaces, gym.utils

  
class Bipedal(URDFBaseRobot):
    def __init__(self):
        """
        load bipedal robot urdf in Pybullet
        TODO: done, implement robot_specific_reset function
        """
        URDFBaseRobot.__init__(self, gravity=-10.0,dt=0.01,file_path="urdf/simbicon_urdf/flame3.urdf")
        # self.reset_sim(disable_velControl=True,disable_gui=False,add_debug=False)

    def robot_specific_reset(self, bullet_client):
        # WalkerBase.robot_specific_reset(self, bullet_client)
        # self.set_initial_orientation(yaw_center=0, yaw_random_spread=np.pi)
        return

    def set_initial_orientation(self, yaw_center, yaw_random_spread):
        # if not self.random_yaw:
        #     yaw = yaw_center
        # else:
        #     yaw = yaw_center + self.np_random.uniform(low=-yaw_random_spread, high=yaw_random_spread)

        # position = [self.start_pos_x, self.start_pos_y, self.start_pos_z + 1.0]
        # orientation = [0, 0, yaw]  # just face random direction, but stay straight otherwise
        # self.robot_body.reset_pose(position, p.getQuaternionFromEuler(orientation))
        # self.initial_z = -1.5 #1.5
        return

if __name__ == "__main__":
    b = Bipedal()
