import numpy as np
import pybullet as p
import time
import pybullet_data
import pickle

def get_thetas():
    """
    return a theta array[1000,7]: theta of centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY
    """
    data_dict = None
    with open('example/data/exp2_trajectory.pkl', 'rb') as f:
        data_dict = pickle.load(f)
    state = data_dict["state_list"] #including vel, theta and collsision
    motor_thetas = state[:,8:22:2]
    return motor_thetas


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.loadURDF("plane.urdf")
humanoid = p.loadURDF("urdf/simbicon_urdf/flame8.urdf",[0, 0, 0.85])
gravId = p.addUserDebugParameter("gravity",0.0,10,-10)
jointIds=[]
paramIds=[]


p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

for j in range (p.getNumJoints(humanoid)):
    info = p.getJointInfo(humanoid,j)
    print(info)

goal_traj = theta_list = get_thetas()
traj_jointIds = [4, 5, 6, 12, 13, 14]
traj_id = 0
p.setRealTimeSimulation(1)
p.getCameraImage(320,200)
p.setGravity(0,0,p.readUserDebugParameter(gravId))
time.sleep(1.0)
while(1):
    traj_id+=1
    p.getCameraImage(320,200)
    p.setGravity(0,0,p.readUserDebugParameter(gravId))
    for i in range(len(traj_jointIds)):
        jointId = traj_jointIds[i]
        p.setJointMotorControl2(humanoid, jointId , p.POSITION_CONTROL, goal_traj[traj_id,i+1], force=140.)
        torso_pos, torso_ori = p.getBasePositionAndOrientation(humanoid)
        base_linVel, base_angVel = p.getBaseVelocity(humanoid)
    # time.sleep(0.1)
