import numpy as np
import pybullet
import time
import pybullet_data


class FlameTorso():
   """
   torso info, including roll, pitch, yaw and angular velocity
   """

   def __init__(self):
       self.roll = 0
       self.rolld = 0
       self.pitch = 0
       self.pitchd = 0
       self.yaw = 0
       self.yawd = 0

   def set_state(self, q, qd):
       self.roll = q[0]
       self.rolld = qd[0]
       self.pitch = q[1]
       self.pitchd = q[1]
       self.yaw = q[2]
       self.yawd = q[2]

class CtrlData():

   def __init__(self):
       self.ref = 0.0
       self.cur = 0.0

   def set_ref(self,ref):
       self.ref = ref

   def set_cur(self,cur):
       self.cur = cur

   def zero(self):
       self.ref = 0.0

class FlameJoint():
   """
   joint state info
   """

   def __init__(self, joint_type="rotate_x"):
       """
       q represents joint angle, qd represents joint velocity
       """
       self.q = 0.0
       # self.y_q = 0.0
       # self.z_q = 0.0
       # self.x_qd = 0.0
       # self.y_qd = 0.0
       self.tauSEA = CtrlData()
       self.tauSEAd = CtrlData()
       self.stiffSEA = 0.0
       self.qd = 0.0
       self.joint_id = 0
       self.type = joint_type

   def set_jointId(self, id):
       """
       """
       self.joint_id = id

   def set_state(self, q, qd):
       """
       every joint has x,y,z axis
       """
       self.q = q
       # self.y_q = y
       # self.z_q = z
       self.qd = qd
       # self.y_qd = vely
       # self.z_qd = velz
       return


class FlameFoot():
   def __init__(self):
       """
       state: 0 means float, 1 means collide with plane
       """
       self.state = 0.0
       self.link_id = 0
       self.front_state = 0
       self.back_state = 0
       return

   def set_state(self, collision_state, front_collision=0, back_collision=0):
       """
       colision state:1  means collision happens on foot
       front_collision: 0 means colision happens on front foot
       back_collision: 0 means colision happens on back foot
       """
       self.state = collision_state
       self.front_state = front_collision
       self.back_state = back_collision
       return

   def set_linkId(self, id):
       self.link_id = id

   def get_state(self):
       return self.state, self.front_state, self.back_state


class PybulletEnv():
   """
   flame environment in self.p
   """

   def __init__(self, gravity=-10.0, dt=0.001,
                file_path="/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame4.urdf"):
       # physics params
       self.g = gravity
       self.dt = dt
       self.p = pybullet
       self.file_path = file_path

       # joint of flame robot
       self.center_hip = FlameJoint(joint_type="rotate_x")  # this joint consist of hipR and hipL
       self.center_hipR = FlameJoint(joint_type="rotate_x")
       self.center_hipL = FlameJoint(joint_type="rotate_x")
       # self.right_hip = FlameJoint(joint_type="rotate_y")
       self.right_hipy = FlameJoint(joint_type="rotate_y")
       # self.right_knee = FlameJoint(joint_type="rotate_y")
       self.right_knee = FlameJoint(joint_type="rotate_y")
       # self.right_ankleY = FlameJoint(joint_type="rotate_y")
       self.right_ankleY = FlameJoint(joint_type="rotate_y")
       self.right_ankleX = FlameJoint(joint_type="rotate_x")
       self.right_foot = FlameFoot()
       # self.left_hip = FlameJoint(joint_type="rotate_y")
       self.left_hipy = FlameJoint(joint_type="rotate_y")
       # self.left_knee = FlameJoint(joint_type="rotate_y")
       self.left_knee = FlameJoint(joint_type="rotate_y")
       # self.left_ankleY = FlameJoint(joint_type="rotate_y")
       self.left_ankleY = FlameJoint(joint_type="rotate_y")
       self.left_ankleX = FlameJoint(joint_type="rotate_x")
       self.left_foot = FlameFoot()
       self.right_hipymot = FlameJoint(joint_type="rotate_z")
       self.left_hipymot = FlameJoint(joint_type="rotate_z")
       self.right_kneemot = FlameJoint(joint_type="rotate_z")
       self.left_kneemot = FlameJoint(joint_type="rotate_z")
       # self.right_ankleYmot = FlameJointSEA(joint_type="rotate_z")
       # self.right_ankleYmot = FlameJointSEA(joint_type="rotate_z")

       # self.joints = [self.center_hipR, self.center_hipL, self.right_hipy, self.right_knee, self.right_ankleY,
       #                self.left_hipy, self.left_knee, self.left_ankleY]

       self.joints = [self.center_hipR,self.center_hipL,self.right_hipy,self.right_knee,self.right_ankleY,self.right_hipymot,self.right_kneemot,
                      self.left_hipy,self.left_knee,self.left_ankleY,self.left_hipymot,self.left_kneemot]

       self.torso = FlameTorso()
       self.bias = 0
       return

   def reset(self, disable_gui=False, disable_velControl=True, add_debug=False):
       if disable_gui:
           self.physics_client = self.p.connect(self.p.DIRECT)
       elif disable_gui == False:
           self.physics_client = self.p.connect(self.p.GUI)
       self.p.resetSimulation()
       self.p.setTimeStep(self.dt)
       self.p.setGravity(0, 0, - self.g)

       # load plane
       self.p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
       self.plane = self.p.loadURDF("plane.urdf")

       # add step down:
       # test_visual = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=[0.2,1,0.1],rgbaColor=[1, 0, 0, 1])
       # test_collision = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=[0.2,1,0.1])
       # test_body = self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
       # baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])

       # add humannoid
       self.humanoid = self.p.loadURDF(self.file_path, [1.0, 1.0, 0.67], useFixedBase=0)
       # self.humanoid = self.p.loadURDF(self.file_path,[0, 0, 0.85])
       # self.humanoid = self.p.loadURDF(self.file_path,[0, 0, 0.85])
       self.p.changeDynamics(self.humanoid, -1, linearDamping=0, angularDamping=0)
       self.p.setGravity(0, 0, self.g)
       # self.p.setGravity(0, 0, 0)

       # disable motors (in order to do control via torques)
       if (disable_velControl):
           for joint in range(self.p.getNumJoints(self.humanoid)):
               self.p.setJointMotorControl2(self.humanoid, joint, self.p.VELOCITY_CONTROL, force=0)

       self.assign_jointId()

       if (add_debug):
           self.gravId = self.p.addUserDebugParameter("gravity", -10, 10, 0)
           self.paramIds, self.jointIds = self.get_motorId()

       # for i in range(20):#+ 10*np.random.randint(low=0, high=20)):
       # self.p.stepSimulation()

       return

   def assign_jointId(self):
       """
       assign joint id to corresponding id
       """
       for j in range(self.p.getNumJoints(self.humanoid)):
           info = self.p.getJointInfo(self.humanoid, j)
           jointName = info[1]
           jointId = info[0]

           if (jointName == b'jointHipR'):
               # print(jointName,jointId)
               self.center_hipR.set_jointId(jointId)

           if (jointName == b'jointHipL'):
               # print(jointName,jointId)
               self.center_hipL.set_jointId(jointId)

           if (jointName == b'jointUpperLegR'):
               # print(jointName,jointId)
               self.right_hipy.set_jointId(jointId)

           if (jointName == b'jointLowerLegR'):
               # print(jointName,jointId)
               self.right_knee.set_jointId(jointId)

           if (jointName == b'jointAnkleR'):
               # print(jointName,jointId)
               self.right_ankleY.set_jointId(jointId)

           if (jointName == b'jointUpperLegL'):
               # print(jointName,jointId)
               self.left_hipy.set_jointId(jointId)

           if (jointName == b'jointLowerLegL'):
               # print(jointName,jointId)
               self.left_knee.set_jointId(jointId)

           if (jointName == b'jointAnkleL'):
               # print(jointName,jointId)
               self.left_ankleY.set_jointId(jointId)

           if (jointName == b'fixed_ankleBridgeL'):
               # print(jointName,jointId)
               self.left_foot.set_linkId(jointId)

           if (jointName == b'fixed_ankleBridgeR'):
               # print(jointName,jointId)
               self.right_foot.set_linkId(jointId)

           if(jointName == b'right_hip_motor'):
               # print(jointName,jointId)
               self.right_hipymot.set_linkId(jointId)

           if(jointName == b'left_hip_motor'):
               # print(jointName,jointId)
               self.left_hipymot.set_linkId(jointId)

           if(jointName == b'right_knee_motor'):
               # print(jointName,jointId)
               self.right_kneemot.set_linkId(jointId)

           if(jointName == b'left_knee_motor'):
               # print(jointName,jointId)
               self.left_kneemot.set_linkId(jointId)

       return

   def step(self, applied_torques, step_sim=True):
       """
       apply torque to the ankle, and then step simulation , then update state of joints and foot,
       applied_torques: list of torques applied to [centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY]
       step_sim: if set to False, you should step simulation outside this function
       """
       centerHip_torque = applied_torques[0]
       self.apply_torque(self.center_hipR, centerHip_torque / 2.0)
       self.apply_torque(self.center_hipL, centerHip_torque / 2.0)

       rightHip_torque = applied_torques[1]
       self.apply_torque(self.right_hipy, rightHip_torque)

       righrKnee_torque = applied_torques[2]
       self.apply_torque(self.right_knee, righrKnee_torque)

       rightAnkleY_torque = applied_torques[3]
       self.apply_torque(self.right_ankleY, rightAnkleY_torque)

       rightHipYMot_torque = applied_torques[4]
       self.apply_torque(self.right_hipymot, rightHipYMot_torque)

       rightKneeMot_torque = applied_torques[5]
       self.apply_torque(self.right_kneemot, rightKneeMot_torque)

       leftHip_torque = applied_torques[6]
       self.apply_torque(self.left_hipy, leftHip_torque)

       leftKnee_torque = applied_torques[7]
       self.apply_torque(self.left_knee, leftKnee_torque)

       leftAnkleY_torque = applied_torques[8]
       self.apply_torque(self.left_ankleY, leftAnkleY_torque)

       leftHipYMot_torque = applied_torques[9]
       self.apply_torque(self.left_hipymot,leftHipYMot_torque)

       leftKneeMot_torque = applied_torques[10]
       self.apply_torque(self.left_kneemot,leftKneeMot_torque)

       # this will pybullet step simulation
       if (step_sim):
           self.p.stepSimulation()

       # this will assign staue value to all joints
       self.update_state()

       return

   def apply_torque(self, joint, torque):
       """
       TODO: force = torque, right?
       Applies given torque at each joint
       """
       self.p.setJointMotorControl2(self.humanoid, joint.joint_id, self.p.TORQUE_CONTROL, force=torque)

   def update_state(self):
       """
       get joint angle and assign it to the corresponding joint
       """
       # update torso angles and angular velocity
       torso_pos, torso_ori = self.p.getBasePositionAndOrientation(self.humanoid)
       torso_angle = self.p.getEulerFromQuaternion(torso_ori)
       torso_linVel, torso_angVel = self.p.getBaseVelocity(self.humanoid)
       self.torso.set_state(torso_angle, torso_angVel)

       # update joint angle of joints
       for joint in self.joints:
           (pos, vel, forces, applied_torque) = self.p.getJointState(self.humanoid, joint.joint_id)
           joint.set_state(q=pos, qd=vel)
       centerHip_q = np.abs(self.center_hipL.q - self.center_hipR.q)
       centerHip_qd = np.abs(self.center_hipL.qd - self.center_hipR.qd)
       self.center_hip.set_state(q=centerHip_q, qd=centerHip_qd)

       self.left_hipy.tauSEA.zero()
       self.right_hipy.tauSEA.zero()

       # update state of foot
       right_foot_collision, right_foot_collision_front, right_foot_collision_back = \
           self.has_contact(self.p, self.humanoid, self.plane, linkA=self.right_foot.link_id, leg_direction='right')
       left_foot_collision, left_foot_collision_front, left_foot_collision_back = \
           self.has_contact(self.p, self.humanoid, self.plane, linkA=self.left_foot.link_id, leg_direction='left')
       self.right_foot.set_state(right_foot_collision, right_foot_collision_front, right_foot_collision_back)
       self.left_foot.set_state(left_foot_collision, left_foot_collision_front, left_foot_collision_back)
       # self.left_foot.set_state(0,left_foot_collision_front,left_foot_collision_back)
       return

   def has_contact(self, bullet_client, bodyA, bodyB, linkA, leg_direction):
       """
       return: 0 means no contact
       ##collision_front: when link pos_X is bigger than contact pos_X, collision happens in the back of link
       ##collision_back: when link pos_X is smaller than contact pos_X, collision happens in the front of link
       Currently, we caculate the relative position of contact point to the local fram, and then decide back and front
       according to the relative position along x axis
       This assumption is based on the robot move along the x axis, if not, the front and back judgement is wrong

       """
       collision = 0
       collision_front = 0
       collision_back = 0
       if len(bullet_client.getContactPoints(bodyA, bodyB, linkIndexA=linkA)) == 0:
           return 0, 0, 0
       else:
           collision = 1
           link_info = bullet_client.getLinkState(bodyA, linkA)
           contact_info = bullet_client.getContactPoints(bodyA, bodyB, linkIndexA=linkA)
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
           return collision, collision_front, collision_back

   def get_motorId(self):
       jointIds = []
       paramIds = []
       jointAngles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0]
       activeJoint = 0
       for j in range(self.p.getNumJoints(self.humanoid)):
           self.p.changeDynamics(self.humanoid, j, linearDamping=0, angularDamping=0)
           info = self.p.getJointInfo(self.humanoid, j)
           jointName = info[1]
           jointType = info[2]
           if (jointType == self.p.JOINT_PRISMATIC or jointType == self.p.JOINT_REVOLUTE):
               activeJoint += 1
               jointIds.append(j)
               paramIds.append(
                   self.p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, jointAngles[activeJoint]))
       return paramIds, jointIds

   def step_debugger(self, step_sim=True):
       """
       read data from debugger
       """
       self.p.getCameraImage(320, 200)
       self.p.setGravity(0, 0, self.p.readUserDebugParameter(self.gravId))

       # joint_states = p.getJointStates(humanoid,range(1))
       info = self.p.getJointInfo(self.humanoid, 0)
       # print(info)
       for i in range(len(self.paramIds)):
           c = self.paramIds[i]
           targetPos = self.p.readUserDebugParameter(c)
           self.p.setJointMotorControl2(self.humanoid, self.jointIds[i], self.p.POSITION_CONTROL, targetPos,
                                        force=140.)
       if (step_sim):
           self.p.stepSimulation()


if __name__ == "__main__":
# dt 0.001
   robot = PybulletEnv(gravity=-10.0, dt=0.001,
                       file_path="/Users/pingy/PycharmProjects/flame_robot/urdf/simbicon_urdf/flame3.urdf")
   robot.reset(disable_velControl= True, add_debug=False)
   for j in range(200):
       robot.p.resetSimulation()
       # robot.reset(disable_velControl=True)
       print(j, end=" ")
       for i in range(20):
           # torques = applied torques
           torque = [1.0, -0.8, -0.8, +0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
           robot.step(torque, step_sim=False)
           robot.p.stepSimulation()
           time.sleep(robot.dt)
   robot.update_state()
   print("center hip q and qd:", robot.center_hip.q, robot.center_hip.qd)
   print("right hip q and qd:", robot.right_hipy.q, robot.right_hipy.qd)
   print("right knee q and qd:", robot.right_knee.q, robot.right_knee.qd)
   print("right ankleY q and qd:", robot.right_ankleY.q, robot.right_ankleY.qd)
   print("left hip q and qd:", robot.left_hipy.q, robot.left_hipy.qd)
   print("left knee q and qd:", robot.left_knee.q, robot.left_knee.qd)
   print("left ankleY q and qd:", robot.left_ankleY.q, robot.left_ankleY.qd)
   print("left foot state:", robot.left_foot.state)
   print("right foot state:", robot.right_foot.state)
   time.sleep(10)