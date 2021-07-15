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
dt = 0.01
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
humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame3.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=1)

# add step down:
test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 0])
test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05])
test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])

hip_left_traj = np.loadtxt("hip_left.csv") # [6001 * 4] matrix, kneeL-11,kneeR-3,HipL-10,HipR-2

traj_id = 0
#disable the default velocity motors
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce = 1
for joint in range(p.getNumJoints(humanoid)):
  p.setJointMotorControl2(humanoid, joint, p.POSITION_CONTROL, force=jointFrictionForce)

for j in range (p.getNumJoints(humanoid)):
    info = p.getJointInfo(humanoid,j)
    print(info)

import time
p.setRealTimeSimulation(0)


while(traj_id<100000):
    p.setGravity(0, 0, GRAVITY)
    jointId = 16
    print(hip_left_traj[traj_id],end="\n")
    p.setJointMotorControl2(humanoid, jointId, p.POSITION_CONTROL, targetPosition=hip_left_traj[traj_id], force=140.)
    p.stepSimulation()
    traj_id += 1
    time.sleep(0.1)
    # time.sleep(1 / 240.)
