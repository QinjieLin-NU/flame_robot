from envs.bipedal_base_env import BipedalBaseEnv
from envs.bipedal_robot import BipedalRobot
# from pybulletgym.envs.roboschool.scenes import StadiumScene


class BipedalBulletEnv(BipedalBaseEnv):
    def __init__(self):
        self.robot = BipedalRobot()
        BipedalBaseEnv.__init__(self, self.robot)

    def robot_specific_reset(self):
        """
        TODO: reset robot on designed positions
        """
        self.robot.robot_specific_reset()
 