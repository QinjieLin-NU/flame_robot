import numpy as np
import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.resetSimulation()
p.loadURDF("plane.urdf")
dt = 0.001
p.setGravity(0,0,-10)
p.setTimeStep(dt)
startPos = [0,0,0.72]
startOrientation = p.getQuaternionFromEuler([0,0,0])
humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame3.urdf", startPos, startOrientation, useFixedBase=0)



# add step down:
test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 0])
test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05])
test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])


# p.setPhysicsEngineParameter(numSolverIterations=100)
# p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

# jointAngles=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# activeJoint=0
# for j in range(p.getNumJoints(humanoid)):
#     p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
#     info = p.getJointInfo(humanoid,j)
#     # print(info)
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

goal_traj = np.loadtxt("allTraj.txt") # [6001 * 4] matrix, kneeL-11,kneeR-3,HipL-10,HipR-2

select_traj = np.zeros((2655,8))
select_traj[:, 0] = goal_traj[:, 2]
select_traj[:, 1] = goal_traj[:, 2]
for i in range(2422):
    select_traj[i, 0] = -goal_traj[i,2]/2
for i in range(2422):
    select_traj[i, 1] = goal_traj[i,2]/2

for i in range(6):
    select_traj[:, 2+i] = goal_traj[:, 3+i]
print(select_traj[100,:])
# traj_jointIds = [15,1,16,2,17,3,18,4]
traj_jointIds = [11,3,12,4,13,5,14,6]
traj_id = 0
# # # p.setRealTimeSimulation(1)
# p.getCameraImage(320,200)
# # time.sleep(5)
# jointFrictionForce = 1
# for joint in range(p.getNumJoints(humanoid)):
#   p.setJointMotorControl2(humanoid, joint, p.POSITION_CONTROL, force=jointFrictionForce)
# while(traj_id<2655):
#     p.setGravity(0,0,-10)
#     print(traj_id,":",end=" ")
#     # forceObj = [0,-1000000,0]
#     # posObj = [0,0.1,0.2]
#     # p.applyExternalForce(humanoid,-1,forceObj,posObj, flags=p.LINK_FRAME)
#     # forceObj2 = [0,1000000,0]
#     # posObj2 = [0,-0.1,0.2]
#     # p.applyExternalForce(humanoid,-1,forceObj2,posObj2, flags=p.LINK_FRAME)
#     # p.getCameraImage(320,200)
#     for i in range(len(traj_jointIds)):
#         jointId = traj_jointIds[i]
#         # print(jointId,goal_traj[traj_id,i],end=" ")
#         p.setJointMotorControl2(humanoid, jointId , p.POSITION_CONTROL, targetPosition=select_traj[traj_id,i], force=140.)
#         print("----------", traj_id, jointId, select_traj[traj_id,i])
#         # p.resetJointState(humanoid, jointId, goal_traj[traj_id,i])
#         # torso_pos, torso_ori = p.getBasePositionAndOrientation(humanoid)
#         # print("torso_pos",torso_pos)
#         # print("torso_ori",torso_ori)
#         # base_linVel, base_angVel = p.getBaseVelocity(humanoid)
#         # print("base_linVel",base_linVel)
#         # print("base_angVel",base_angVel)
#     # print()
#     p.stepSimulation()
#     traj_id += 10
#     time.sleep(0.01)
# # while(1):
# #     p.getCameraImage(320,200)
# #     p.setGravity(0,0,p.readUserDebugParameter(gravId))
