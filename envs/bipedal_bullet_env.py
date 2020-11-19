from envs.bipedal_base_env import BipedalBaseEnv
from envs.bipedal_robot import Bipedal
# from pybulletgym.envs.roboschool.scenes import StadiumScene


class BipedalBulletEnv(BipedalBaseEnv):
    def __init__(self):
        self.robot = Bipedal()
        BipedalBaseEnv.__init__(self, self.robot)

    def robot_specific_reset(self):
        """
        TODO: reset robot on designed positions
        """
        self.robot.robot_specific_reset()
 