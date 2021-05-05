import numpy as np
import pybullet as p
import time
import pybullet_data
import time


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.loadURDF("plane.urdf")
humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/test.urdf",[0, 0, 0.85], useFixedBase=0)

jointIds=[]
paramIds=[]

p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

i = 0

p.setRealTimeSimulation(1)
p.getCameraImage(320,200)
p.setGravity(0,0,-10)
# time.sleep(1.0)

for j in range(p.getNumJoints(humanoid)):
   info = p.getJointInfo(humanoid, j)
   jointName = info[1]
   jointId = info[0]

   if (jointName == b'jointLowerLegR'):
       knee_jointId = jointId

while 1:
   # print(traj_id,":",end=" ")
   torque = -90.0
   p.setJointMotorControl2(humanoid, knee_jointId, p.TORQUE_CONTROL, force=torque)
   torso_pos, torso_ori = p.getBasePositionAndOrientation(humanoid)
   print("torso_pos", torso_pos)
   print("torso_ori", torso_ori)
   (pos, vel, forces, applied_torque) = p.getJointState(humanoid, knee_jointId)
   print("joint position", pos)
   print("joint velocity", vel)

   # time.sleep(0.01)