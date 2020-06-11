from envs.pybullet_env import PybulletEnv
import time
from calTorque import multijointController

class bipedal_EActrl():
    def __init__(self, weights):
        self.weights = weights

        self.robot = PybulletEnv(gravity=-10.0,dt=0.01,file_path="urdf/simbicon_urdf/flame3.urdf")
        self.robot.reset(disable_velControl=True,add_debug=False)
        self.controller = multijointController(robot,weights)

    def fitness(self, torques):
        C1 = torques[0]*dt#centerHip_torque
        C2 = torques[1]*dt#rightHip_torque
        C3 = torques[2]*dt#rightKnee_torque
        C4 = torques[3]*dt#rightAnkleY_torque
        C5 = torques[4]*dt#leftHip_torque
        C6 = torques[5]*dt#leftKnee_torque
        C7 = torques[6]*dt#leftAnkleY_torque
        Cost = C1+C2+C3+C4+C5+C6+C7
        return Cost

    def move(self):

        i=0
        time.sleep(2.0)

        fall_flag = False
        fitness = 0


        while(i<100000):
            #calculate torques and apply torques to robots
            torques = controller.update()

            robot.step(torques,step_sim=False)
            # robot.step(step_sim=False)
            
            #step simulation id needed and update state of robot 
            robot.p.stepSimulation()

            time.sleep(robot.dt)
            robot.update_state()

            #EA
            if (fall_flag==False):
                fitness += fitness(torques)
            else:
                break

            i+=1

        return fitness






    