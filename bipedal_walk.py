import numpy as np
import pybullet as p
import time
import pybullet_data


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.loadURDF("plane.urdf")
humanoid = p.loadURDF("urdf/simbicon_urdf/flame3.urdf",[0, 0, 0.85])
gravId = p.addUserDebugParameter("gravity",-10,10,-10)
jointIds=[]
paramIds=[]

#add step down:
test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 1])
test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05])
test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])


p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

# jointAngles=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# activeJoint=0
# for j in range (p.getNumJoints(humanoid)):
#     p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
#     info = p.getJointInfo(humanoid,j)
#     print(info)
#     jointName = info[1]
#     jointType = info[2]
#     if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
#         activeJoint+=1
#         jointIds.append(j)
#         paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,jointAngles[activeJoint]))
#         # p.resetJointState(humanoid, j, jointAngles[activeJoint])

for j in range (p.getNumJoints(humanoid)):
    info = p.getJointInfo(humanoid,j)
    print(info)

goal_traj = np.loadtxt("trajectories.txt") # [6001 * 4] matrix, kneeL-11,kneeR-3,HipL-10,HipR-2
traj_jointIds = [11,3,10,2]
traj_id = 0
p.setRealTimeSimulation(1)
p.getCameraImage(320,200)
p.setGravity(0,0,p.readUserDebugParameter(gravId))
time.sleep(1.0)
while(1):
    # print(traj_id,":",end=" ")
    traj_id +=100
    p.getCameraImage(320,200)
    p.setGravity(0,0,p.readUserDebugParameter(gravId))
    for i in range(len(traj_jointIds)):
        jointId = traj_jointIds[i]
        # print(jointId,goal_traj[traj_id,i],end=" ")
        p.setJointMotorControl2(humanoid, jointId , p.POSITION_CONTROL, goal_traj[traj_id,i], force=140.)
        # p.resetJointState(humanoid, jointId, goal_traj[traj_id,i])
        torso_pos, torso_ori = p.getBasePositionAndOrientation(humanoid)
        print("torso_pos",torso_pos)
        print("torso_ori",torso_ori)
        base_linVel, base_angVel = p.getBaseVelocity(humanoid)
        print("base_linVel",base_linVel)
        print("base_angVel",base_angVel)
    # print()
    time.sleep(0.01)
# while(1):
#     p.getCameraImage(320,200)
#     p.setGravity(0,0,p.readUserDebugParameter(gravId))
