import numpy as np
import pybullet as p
import time
import pybullet_data
import time
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.resetSimulation()
p.loadURDF("plane.urdf")

startPos = [0,0,0.85]
startOrientation = p.getQuaternionFromEuler([0,0,0])

humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/test.urdf", startPos, startOrientation, useFixedBase=1)
# 0.85
jointIds=[]
paramIds=[]

frictionId = p.addUserDebugParameter("jointFriction", 0, 20, 10)
torqueId = p.addUserDebugParameter("joint torque", 0, 2000, 1000)
# p.setPhysicsEngineParameter(numSolverIterations=100)
# p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

dt = 0.01

# p.setRealTimeSimulation(1)
# p.getCameraImage(320,200)
p.setGravity(0,0,-10)
p.setTimeStep(dt)

allData = []
allData2 = []
i = 0
for j in range(p.getNumJoints(humanoid)):
   info = p.getJointInfo(humanoid, j)
   jointName = info[1]
   jointId = info[0]
   jointType = info[2]
   print("joint------", j, jointId,jointName, jointType)
   if (jointName == b'jointLowerLegR'):
       knee_jointId = jointId
   if (jointName == b'hingeToMotor'):
       hingeId = jointId


while i<50:

   # frictionForce = p.readUserDebugParameter(frictionId)
   # jointTorque = p.readUserDebugParameter(torqueId)
   # frictionForce = 0.096
   # jointTorque = 50
   # # # set the joint friction
   # p.setJointMotorControl2(humanoid, knee_jointId, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
   # p.setJointMotorControl2(humanoid, knee_jointId, p.TORQUE_CONTROL, force=jointTorque)

   frictionForce2 = 0.0
   jointTorque2 = 10
   # set the joint friction
   p.setJointMotorControl2(humanoid, hingeId, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce2)
   p.setJointMotorControl2(humanoid, hingeId, p.TORQUE_CONTROL, force=jointTorque2)
   # p.stepSimulation()
   # time.sleep(0.01)
   # torque = 1000.0
   # p.setJointMotorControl2(humanoid, knee_jointId, p.TORQUE_CONTROL, force=torque)
   p.stepSimulation()
   time.sleep(0.01)
   (pos, vel, forces, applied_torque) = p.getJointState(humanoid, knee_jointId)
   print("joint position", pos)
   print("joint velocity", vel)
   (pos2, vel2, forces2, applied_torque2) = p.getJointState(humanoid, hingeId)
   print("hinge_joint position", pos2)
   print("hinge_joint velocity", vel2)
   # print("link vel",worldLinkAngularVelocity )
   allData.append(vel)
   allData2.append(vel2)
   i=i+1

# frictionForce2 = 0.0
# jointTorque2 = 50
# # set the joint friction
# p.setJointMotorControl2(humanoid, hingeId, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce2)
#
# p.setJointMotorControl2(humanoid, hingeId, p.TORQUE_CONTROL, force=jointTorque2)
# p.stepSimulation()
# time.sleep(0.01)
# (pos2, vel2, forces2, applied_torque2) = p.getJointState(humanoid, hingeId)
# (pos, vel, forces, applied_torque) = p.getJointState(humanoid, knee_jointId)
# print("joint position", pos)
# print("joint velocity", vel)
# print("hinge_joint position", pos2)
# print("hinge_joint velocity", vel2)
#
# diff = pos2 - pos
#
# frictionForce = 0.096
# jointTorque = diff*10
# # # set the joint friction
# p.setJointMotorControl2(humanoid, knee_jointId, p.VELOCITY_CONTROL, targetVelocity=0, force=frictionForce)
# p.setJointMotorControl2(humanoid, knee_jointId, p.TORQUE_CONTROL, force=jointTorque)
# p.setJointMotorControl2(humanoid, hingeId, p.TORQUE_CONTROL, force=0)
# p.stepSimulation()
# time.sleep(0.01)
#
xpoint = range(50)
plt.plot(xpoint, allData, label = 'right hipy')
plt.plot(xpoint, allData2, label = 'right hinge')
plt.show()
# (pos2, vel2, forces2, applied_torque2) = p.getJointState(humanoid, hingeId)
# (pos, vel, forces, applied_torque) = p.getJointState(humanoid, knee_jointId)
# print("joint position", pos)
# print("joint velocity", vel)
# print("hinge_joint position", pos2)
# print("hinge_joint velocity", vel2)
# time.sleep(10000)