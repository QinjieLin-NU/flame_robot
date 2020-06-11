from envs.pybullet_env import PybulletEnv
import time
from calTorque import multijointController
from EA import fitness

class EA_ctrl():
    def __init__(weights):
        self.weights = weights

        self.robot = PybulletEnv(gravity=-10.0,dt=0.01,file_path="urdf/simbicon_urdf/flame3.urdf")
        self.robot.reset(disable_velControl=True,add_debug=False)
        self.controller = multijointController(robot,weights)

    def move():

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






    