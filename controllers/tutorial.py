# import pybullet as p
# import time
# import pybullet_data
import matplotlib.pyplot as plt
# dt = 1e-3
# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.resetSimulation()
# p.setGravity(0,0,-10)
# p.setTimeStep(dt)
# planeId = p.loadURDF("plane.urdf")
# startPos = [0,0,1]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
#
# frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
# torqueId = p.addUserDebugParameter("joint torque", 0, 20, 5)
#
# for j in range(p.getNumJoints(boxId)):
#    info = p.getJointInfo(boxId, j)
#    jointName = info[1]
#    jointId = info[0]
#    jointType = info[2]
#    print("joint", j, jointId,jointName, jointType)
#    if (jointName == b'base_to_right_leg'):
#        joint0 = jointId
#    if (jointName == b'right_base_joint'):
#        joint1 = jointId
#    if (jointName == b'right_front_wheel_joint'):
#        joint2 = jointId
#
# i = 0
# allData = []
# while i<2000:
#    # print(traj_id,":",end=" ")
#    # maxForce = 0
#    # mode = p.VELOCITY_CONTROL
#    # p.setJointMotorControl2(boxId,joint2,controlMode=mode,force=maxForce)
#    frictionForce = p.readUserDebugParameter(frictionId)
#    jointTorque = p.readUserDebugParameter(torqueId)
#    # set the joint friction
#    p.setJointMotorControl2(boxId, 11, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
#    # torque = 1000.0
#    # p.setJointMotorControl2(boxId, joint2, controlMode=p.TORQUE_CONTROL, force=torque)
#    # p.stepSimulation()
#    # time.sleep(dt)
#    p.setJointMotorControl2(boxId, 11, p.TORQUE_CONTROL, force=jointTorque)
#    p.stepSimulation()
#    time.sleep(0.01)
#    torso_pos, torso_ori = p.getBasePositionAndOrientation(boxId)
#    print("torso_pos", torso_pos)
#    print("torso_ori", torso_ori)
#    (pos, vel, forces, applied_torque) = p.getJointState(boxId, joint2)
#    # (linkWorldPosi, linkWorldOri, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition,worldLinkFrameOrientation,worldLinkLinearVelocity,worldLinkAngularVelocity) = p.getLinkStates(humanoid, knee_jointId)
#    print("joint position", pos)
#    print("joint velocity", vel)
#    # print("link vel",worldLinkAngularVelocity )
#    allData.append(vel)
#    i=i+1
#
#
# xpoint = range(2000)
# plt.plot(xpoint, allData, label = 'right hipy')
# plt.show()





import pybullet as p
import numpy as np
import pybullet_data
import os
import time
GRAVITY = -9.8
dt = 0.0001
iters = 2000
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
#p.setRealTimeSimulation(True)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.86]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame5.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=0)

# add step down:
test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 0])
test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05])
test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])

goal_traj = np.loadtxt("/Users/pingy/PycharmProjects/flame_robot/traj_control/allTraj.txt") # [6001 * 4] matrix, kneeL-11,kneeR-3,HipL-10,HipR-2

select_traj = np.zeros((2655,8))
select_traj[:, 0] = goal_traj[:, 2]
select_traj[:, 1] = goal_traj[:, 2]
for i in range(2422):
    select_traj[i, 0] = -goal_traj[i,2]/2
for i in range(2422):
    select_traj[i, 1] = goal_traj[i,2]/2

for i in range(6):
    select_traj[:, 2+i] = goal_traj[:, 3+i]

# traj_jointIds = [15,1,16,2,17,3,18,4]
traj_jointIds = [11,3,12,4,13,5,14,6]
traj_id = 0
#disable the default velocity motors
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce = 1
for joint in range(p.getNumJoints(humanoid)):
  p.setJointMotorControl2(humanoid, joint, p.POSITION_CONTROL, force=jointFrictionForce)

for j in range (p.getNumJoints(humanoid)):
    info = p.getJointInfo(humanoid,j)
    print(info)
#for i in range(10000):
#     p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#     p.stepSimulation()
#import ipdb
#ipdb.set_trace()
import time
p.setRealTimeSimulation(0)
# while (1):
#   #p.stepSimulation()
#   #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#   p.setGravity(0, 0, GRAVITY)
#   time.sleep(1 / 240.)
# time.sleep(1000)
posx = []
posy = []
array = []
while(traj_id<1500):
    p.setGravity(0, 0, GRAVITY)
    # print(traj_id,":",end=" ")
    # forceObj = [0,-1000000,0]
    # posObj = [0,0.1,0.2]
    # p.applyExternalForce(humanoid,-1,forceObj,posObj, flags=p.LINK_FRAME)
    # forceObj2 = [0,1000000,0]
    # posObj2 = [0,-0.1,0.2]
    # p.applyExternalForce(humanoid,-1,forceObj2,posObj2, flags=p.LINK_FRAME)
    # p.getCameraImage(320,200)
    for i in range(len(traj_jointIds)):
        jointId = traj_jointIds[i]
        # print(jointId,goal_traj[traj_id,i],end=" ")
        p.setJointMotorControl2(humanoid, jointId , p.POSITION_CONTROL, targetPosition=select_traj[traj_id,i], force=140.)
        # print("----------", traj_id, jointId, select_traj[traj_id,i])
        p.stepSimulation()

        # p.resetJointState(humanoid, jointId, goal_traj[traj_id,i])
        # torso_pos, torso_ori = p.getBasePositionAndOrientation(humanoid)
        # print("torso_pos",torso_pos)
        # print("torso_ori",torso_ori)
        # base_linVel, base_angVel = p.getBaseVelocity(humanoid)
        # print("base_linVel",base_linVel)
        # print("base_angVel",base_angVel)
    # print()
        # link index2:body,index3:hipCylinderR
    array = p.getLinkState(bodyUniqueId=humanoid, linkIndex=2)
    linkWorldPos = array[0]
    print("<<<<<<<<linkworldPosition:", traj_id)
    posx.append(linkWorldPos[0])
    posy.append(linkWorldPos[1])
    # p.stepSimulation()
    traj_id += 1
    time.sleep(0.001)
    # time.sleep(1 / 240.)
xpoint = range(1500)
plt.plot(xpoint, posx, label = 'x')
plt.plot(xpoint, posy, label = 'y')
plt.show()