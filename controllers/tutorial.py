# import pybullet as p
# import time
# import pybullet_data
# import matplotlib.pyplot as plt
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
import pybullet_data
import os
import time
GRAVITY = -9.8
dt = 1e-3
iters = 2000
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
#p.setRealTimeSimulation(True)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1.13]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
botId = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame4.urdf", cubeStartPos, cubeStartOrientation)

#disable the default velocity motors
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce = 1
for joint in range(p.getNumJoints(botId)):
  p.setJointMotorControl2(botId, joint, p.POSITION_CONTROL, force=jointFrictionForce)

#for i in range(10000):
#     p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
#     p.stepSimulation()
#import ipdb
#ipdb.set_trace()
import time
p.setRealTimeSimulation(1)
while (1):
  #p.stepSimulation()
  #p.setJointMotorControl2(botId, 1, p.TORQUE_CONTROL, force=1098.0)
  p.setGravity(0, 0, GRAVITY)
  time.sleep(1 / 240.)
time.sleep(1000)