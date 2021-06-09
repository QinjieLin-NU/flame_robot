import time
# from calTorque import EA_weights_Controller
import sys

sys.path.append("..")
from envs.pybullet_env import PybulletEnv
from controllers.EA_controller import EA_weights_Controller

class bipedal_EActrl():
    def __init__(self, weights):
        self.weights = weights
        self.dt = 0.01

        self.robot = PybulletEnv(gravity=-10.0, dt=self.dt, file_path="../urdf/simbicon_urdf/flame5.urdf")
        self.robot.reset(disable_velControl=True, add_debug=False)
        self.controller = EA_weights_Controller(self.robot)

    def Accum_Torques(self, torques):
        C1 = abs(torques[0]) * self.dt  # centerHip_torque
        C2 = abs(torques[1]) * self.dt  # rightHip_torque
        C3 = abs(torques[2]) * self.dt  # rightKnee_torque
        C4 = abs(torques[3]) * self.dt  # rightAnkleY_torque
        C5 = abs(torques[4]) * self.dt  # leftHip_torque
        C6 = abs(torques[5]) * self.dt  # leftKnee_torque
        C7 = abs(torques[6]) * self.dt  # leftAnkleY_torque
        accum_torque = C1 + C2 + C3 + C4 + C5 + C6 + C7
        return accum_torque

    def fitness(self, torques):
        multiplier = 1.0
        max_dist = 0
        max_torque = 0

        distTraveled = getDistTraveled()

        if distTraveled > max_dist:
            energy_remaining = max_torque - accum_torque
            energy_per_meter = accum_torque / max_dist
            distTraveled = max_dist + (energy_remaining / energy_per_meter)

        return distTraveled * multiplier

    def move(self):

        i = 0
        time.sleep(2.0)

        fall_flag = False
        fitness = 0

        while (i < 100000):
            # calculate torques and apply torques to robots
            torques = controller.update()

            robot.step(torques, step_sim=False)
            # robot.step(step_sim=False)

            # step simulation id needed and update state of robot
            robot.p.stepSimulation()

            time.sleep(robot.dt)
            robot.update_state()

            # EA
            if (fall_flag == False):
                accum_torque += Accum_Torques(torques)
            else:
                break

            i += 1

        return accum_torque
