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
from envs.pybullet_env import PybulletEnv

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
#p.setRealTimeSimulation(True)
p.setGravity(0, 0, GRAVITY)
p.setTimeStep(dt)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.8]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
humanoid = p.loadURDF("/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame5.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=0)
# humanoid2 = PybulletEnv(GRAVITY,dt)
# # add step down:
# test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 0])
# test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.05])
# test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])

knee_left_traj = np.loadtxt("knee_left.csv") # 13
knee_right_traj = np.loadtxt("knee_right.csv") # 5
hip_left_traj = np.loadtxt("hip_left.csv") # 12
hip_right_traj = np.loadtxt("hip_right.csv") # 4
ankle_left_traj = np.loadtxt("ankle_left.csv") # 14
ankle_right_traj = np.loadtxt("ankle_right.csv") # 6

traj_id = 0
#disable the default velocity motors
#and set some position control with small force to emulate joint friction/return to a rest pose
jointFrictionForce = 500
for joint in range(p.getNumJoints(humanoid)):
  p.setJointMotorControl2(humanoid, joint, p.POSITION_CONTROL, force=jointFrictionForce)

for j in range (p.getNumJoints(humanoid)):
    info = p.getJointInfo(humanoid,j)
    print(info)

import time
p.setRealTimeSimulation(0)


def has_contact(bullet_client, linkA):
    """
    return: 0 means no contact
    ##collision_front: when link pos_X is bigger than contact pos_X, collision happens in the back of link
    ##collision_back: when link pos_X is smaller than contact pos_X, collision happens in the front of link
    Currently, we calculate the relative position of contact point to the local fram, and then decide back and front
    according to the relative position along x axis
    This assumption is based on the robot move along the x axis, if not, the front and back judgement is wrong

    """
    collision = 0
    collision_front = 0
    collision_back = 0
    if len(bullet_client.getContactPoints(humanoid, planeId, linkIndexA=linkA)) == 0:
        return 0
    else:
        collision = 1
        link_info = bullet_client.getLinkState(humanoid, linkA)
        contact_info = bullet_client.getContactPoints(humanoid, planeId, linkIndexA=linkA)
        link_pos = link_info[0]
        link_quar = link_info[1]
        contact_posOnA = contact_info[0][5]
        contact_qua = (1, 0, 0, 0)
        link_pos_invert, link_quar_invert = bullet_client.invertTransform(link_pos, link_quar)
        rel_pos, rel_qua = bullet_client.multiplyTransforms(link_pos_invert, link_quar_invert, contact_posOnA,
                                                            contact_qua)
        # print("relative position: ",rel_pos)
        if (rel_pos[0] > 0):
            collision_front = 1
        else:
            collision_back = 1
        # this is the second version judgment
        # if((link_pos[0] - contact_posOnA[0])> 0):
        #     collision_back = 1
        # else:
        #     collision_front = 1
        # print("link world position :",link_info[0],"contact point world position",contact_posOnA)
        # this is the first verision judgment
        # joint_angle = self.__dict__[leg_direction+'_ankleY'].q
        # if(joint_angle>=0):
        #     collision_front =1
        # else:
        #     collision_back = 1
        return collision

left_foot_list = []
right_foot_list = []
while(traj_id<15000):
    p.setGravity(0, 0, GRAVITY)
    hip_right_Id = 4
    hip_left_Id = 12
    knee_left_id = 13
    knee_right_id = 5
    ankle_left_id = 14
    ankle_right_id = 6

    p.setJointMotorControl2(humanoid, hip_left_Id, p.POSITION_CONTROL, targetPosition=hip_left_traj[traj_id], force=140.)
    p.setJointMotorControl2(humanoid, hip_right_Id, p.POSITION_CONTROL, targetPosition=hip_right_traj[traj_id], force=140.)
    p.setJointMotorControl2(humanoid, knee_left_id, p.POSITION_CONTROL, targetPosition=knee_left_traj[traj_id], force=140.)
    p.setJointMotorControl2(humanoid, knee_right_id, p.POSITION_CONTROL, targetPosition=knee_right_traj[traj_id], force=140.)
    p.setJointMotorControl2(humanoid, ankle_left_id, p.POSITION_CONTROL, targetPosition=ankle_left_traj[traj_id], force=140.)
    p.setJointMotorControl2(humanoid, ankle_right_id, p.POSITION_CONTROL, targetPosition=ankle_right_traj[traj_id], force=140.)

    p.stepSimulation()
    traj_id += 1
    time.sleep(0.01)
    left_foot= has_contact(p,15)
    left_foot_list.append(left_foot)
    right_foot = has_contact(p,7)
    right_foot_list.append(right_foot)
    if left_foot==0 and right_foot==0:
        print("both 0!!!!!!")
    if left_foot==1 and right_foot==1:
        print("both 1!!!!!!====================")
    print("right foot:",has_contact(p,7))
    print("left_foot:",has_contact(p,15))
    # time.sleep(1 / 240.)

xpoint = range(15000)
plt.plot(xpoint, left_foot_list, label = 'x')
plt.plot(xpoint, right_foot_list, label = 'y')
plt.show()
