import numpy as np
import pybullet
import time
import pybullet_data

class FlameJoint():
    """
    joint state info
    """
    def __init__(self,joint_type="rotate_x"):
        """
        q represents joint angle, qd represents joint velocity
        """
        self.q = 0.0
        # self.y_q = 0.0
        # self.z_q = 0.0
        # self.x_qd = 0.0
        # self.y_qd = 0.0
        self.qd = 0.0
        self.joint_id = 0
        self.type = joint_type

    def set_jointId(self,id):
        """
        """
        self.joint_id = id

    def set_state(self,q,qd):
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
        return

    def set_state(self,collision_state):
        self.state = collision_state
        return
    
    def set_linkId(self,id):
        self.link_id = id

    

class PybulletEnv():
    """
    flame environment in self.p
    """
    def __init__(self,gravity=-10.0,dt=0.01,file_path="../urdf/simbicon_urdf/flame3.urdf"):
        #physics params
        self.g = gravity
        self.dt = dt
        self.p = pybullet
        self.file_path = file_path

        #joint of flame robot
        self.center_hip= FlameJoint(joint_type="rotate_x") # this joint consist of hipR and hipL
        self.center_hipR = FlameJoint(joint_type="rotate_x")
        self.center_hipL = FlameJoint(joint_type="rotate_x")
        self.right_hip = FlameJoint(joint_type="rotate_y")
        self.right_knee = FlameJoint(joint_type="rotate_y")
        self.right_ankleY = FlameJoint(joint_type="rotate_y")
        self.right_ankleX = FlameJoint(joint_type="rotate_x")
        self.right_foot = FlameFoot()
        self.left_hip = FlameJoint(joint_type="rotate_y")
        self.left_knee = FlameJoint(joint_type="rotate_y")
        self.left_ankleY = FlameJoint(joint_type="rotate_y")
        self.left_ankleX = FlameJoint(joint_type="rotate_x")
        self.left_foot = FlameFoot()

        self.joints = [self.center_hipR,self.center_hipL,self.right_hip,self.right_knee,self.right_ankleY,self.left_hip,self.left_knee,self.left_ankleY]
        return


    def reset(self,disable_gui=False,disable_velControl=True):
        if disable_gui:
            self.physics_client = self.p.connect(self.p.DIRECT)
        elif disable_gui==False:
            self.physics_client = self.p.connect(self.p.GUI)
        self.p.resetSimulation()
        self.p.setTimeStep(self.dt)
        self.p.setGravity(0,0,self.g)

        #load plane
        self.p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
        self.plane = self.p.loadURDF("plane.urdf")

        #add step down:
        test_visual = self.p.createVisualShape(self.p.GEOM_BOX, halfExtents=[0.2,1,0.05],rgbaColor=[1, 0, 0, 1])
        test_collision = self.p.createCollisionShape(self.p.GEOM_BOX, halfExtents=[0.2,1,0.05])
        test_body = self.p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
        baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])

        #add humannoid
        self.humanoid = self.p.loadURDF(self.file_path,[0, 0, 0.85])
        # self.humanoid = self.p.loadURDF("../urdf/simbicon_urdf/flame3.urdf",[0.1, 0, 1.95])
        self.p.changeDynamics(self.humanoid,-1,linearDamping=0, angularDamping=0)
        self.p.setGravity(0,0,self.g)

        #disable motors (in order to do control via torques)
        if(disable_velControl):
            for joint in range(self.p.getNumJoints(self.humanoid)):
                self.p.setJointMotorControl2(self.humanoid, joint, self.p.VELOCITY_CONTROL, force=0)

        self.assign_jointId()

        # for i in range(20):#+ 10*np.random.randint(low=0, high=20)):
            # self.p.stepSimulation()

        return

    def assign_jointId(self):
        """
        assign joint id to corresponding id
        """
        for j in range (self.p.getNumJoints(self.humanoid)):
            info = self.p.getJointInfo(self.humanoid,j)
            jointName = info[1]
            jointId = info[0]

            print(info)

            if(jointName == 'jointHipR'):
                self.center_hipR.set_jointId(jointId)
            
            if(jointName == 'jointHipL'):
                self.center_hipL.set_jointId(jointId)

            if(jointName == 'jointUpperLegR'):
                self.right_hip.set_jointId(jointId)

            if(jointName == 'jointLowerLegR'):
                self.right_knee.set_jointId(jointId)

            if(jointName == 'jointAnkleR'):
                self.right_ankleY.set_jointId(jointId)

            if(jointName == 'jointUpperLegL'):
                self.left_hip.set_jointId(jointId)

            if(jointName == 'jointLowerLegL'):
                self.left_knee.set_jointId(jointId)

            if(jointName == 'jointAnkleL'):
                self.left_ankleY.set_jointId(jointId) 

            if(jointName == 'fixed_ankleBridgeL'):
                self.left_foot.set_linkId(jointId)

            if(jointName == 'fixed_ankleBridgeR'):
                self.right_foot.set_linkId(jointId)

        return

    def step(self,applied_torques,step_sim = True):
        """
        apply torque to the ankle, and then step simulation , then update state of joints and foot,
        applied_torques: list of torques applied to [centerHip,RHip,RKnee,RAnkleY,LHip,LKnee,LAnkleY]
        step_sim: if set to False, you shoud step simulation outside this function
        """
        centerHip_torque = applied_torques[0]
        self.apply_torque(self.center_hipR,centerHip_torque/2.0)
        self.apply_torque(self.center_hipL,centerHip_torque/2.0)

        rightHip_torque = applied_torques[1]
        self.apply_torque(self.right_hip,rightHip_torque)

        righrKnee_torque = applied_torques[2]
        self.apply_torque(self.right_knee,righrKnee_torque)

        rightAnkleY_torque = applied_torques[3]
        self.apply_torque(self.right_ankleY,rightAnkleY_torque)

        leftHip_torque = applied_torques[4]
        self.apply_torque(self.left_hip,leftHip_torque)

        leftKnee_torque = applied_torques[5]
        self.apply_torque(self.left_knee,leftKnee_torque)

        leftAnkleY_torque = applied_torques[6]
        self.apply_torque(self.left_ankleY,leftAnkleY_torque)

        # this will pybullet step simulation
        if(step_sim):
            self.p.stepSimulation()

        #this will assign staue value to all joints
        self.update_state()

        return


    def apply_torque(self, joint, torque):
        """
        TODO: force = torque, right?
        Applies given torque at each joint
        """
        self.p.setJointMotorControl2(self.humanoid, joint.joint_id, self.p.TORQUE_CONTROL,force=torque)

    def update_state(self):
        """
        get joint angle and assign it to the corresponding joint
        """
        # update joint angle of joints
        for joint in self.joints:
            (pos,vel,forces,applied_torque) = self.p.getJointState(self.humanoid,joint.joint_id)
            joint.set_state(q=pos,qd=vel)
        centerHip_q = np.abs(self.center_hipL.q-self.center_hipR.q)
        centerHip_qd = np.abs(self.center_hipL.qd - self.center_hipR.qd)
        self.center_hip.set_state(q=centerHip_q,qd=centerHip_qd)
        
        #update state of foot
        right_foot_collision = self.has_contact(self.p,self.humanoid,self.plane,linkA=self.right_foot.link_id)
        left_foot_collision = self.has_contact(self.p,self.humanoid,self.plane,linkA=self.left_foot.link_id)
        self.right_foot.set_state(right_foot_collision)
        self.left_foot.set_state(left_foot_collision)
        return
    
    def has_contact(self, bullet_client, bodyA, bodyB, linkA):
        """
        return: 0 means no contact
        """
        if len(bullet_client.getContactPoints(bodyA,bodyB, linkIndexA=linkA))==0:
            return 0
        else:
            print(bullet_client.getContactPoints(bodyA,bodyB, linkIndexA=linkA))
            return 1



if __name__ == "__main__":
    robot = PybulletEnv(gravity=-10.0,dt=0.01)
    robot.reset(disable_velControl=True)
    for i in range(200):
        #torques = applied torques
        torques = [-1.0,-0.8,-0.8,+0.8,0.8,0.8,0.8]
        # robot.step(torque,step_sim=False)
        robot.p.stepSimulation()
        time.sleep(robot.dt)
    robot.update_state()
    print("center hip q and qd:",robot.center_hip.q,robot.center_hip.qd)
    print("right hip q and qd:",robot.right_hip.q,robot.right_hip.qd)
    print("right knee q and qd:",robot.right_knee.q,robot.right_knee.qd)
    print("right ankleY q and qd:",robot.right_ankleY.q,robot.right_ankleY.qd)
    print("left hip q and qd:",robot.left_hip.q,robot.left_hip.qd)
    print("left knee q and qd:",robot.left_knee.q,robot.left_knee.qd)
    print("left ankleY q and qd:",robot.left_ankleY.q,robot.left_ankleY.qd)
    print("left foot state:",robot.left_foot.state)
    print("right foot state:",robot.right_foot.state)
    time.sleep(10)
