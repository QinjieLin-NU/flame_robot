from envs.pybullet_env import PybulletEnv
import time
from calTorque import cal_Torque

if __name__ == "__main__":
    robot = PybulletEnv(gravity=-10.0,dt=0.01,file_path="urdf/simbicon_urdf/flame3.urdf")
    robot.reset(disable_velControl=True)
    i=0
    while(i<10):
        #torques = applied torques
        # torques = [-1.0,-0.8,-0.8,+0.8,0.8,0.8,0.8]
        torques = cal_Torque(robot)
        print("torques",torques)
        robot.step(torques,step_sim=False)
        robot.p.stepSimulation()
        time.sleep(robot.dt)
        robot.update_state()
        i+=1
    print("center hip q and qd:",robot.center_hip.q,robot.center_hip.qd)
    print("right hip q and qd:",robot.right_hip.q,robot.right_hip.qd)
    print("right knee q and qd:",robot.right_knee.q,robot.right_knee.qd)
    print("right ankleY q and qd:",robot.right_ankleY.q,robot.right_ankleY.qd)
    print("left hip q and qd:",robot.left_hip.q,robot.left_hip.qd)
    print("left knee q and qd:",robot.left_knee.q,robot.left_knee.qd)
    print("left ankleY q and qd:",robot.left_ankleY.q,robot.left_ankleY.qd)
    print("left foot state:",robot.left_foot.state)
    print("right foot state:",robot.right_foot.state)


    time.sleep(10)