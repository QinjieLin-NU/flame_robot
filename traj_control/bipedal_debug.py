import pybullet as p
import time
import pybullet_data
import pybullet_envs


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf
p.loadURDF("plane.urdf")
# p.loadURDF("urdf/plane.urdf",basePosition=[0,0,-1.5])
# humanoid = p.loadURDF("cassie/urdf/cassie_collide.urdf",[0,0,0.8], useFixedBase=False)
# humanoid = p.loadURDF("urdf/simbicon_urdf/biped2d.urdf",[0, 0, 1.2])
# humanoid = p.loadURDF("urdf/simbicon_urdf/humanoid_nohead.urdf",[0, 0, 0.31])
# humanoid = p.loadURDF("urdf/simbicon_urdf/flame.urdf",[0, 0, 1.0])
humanoid = p.loadURDF("urdf/simbicon_urdf/flame7.urdf",[0, 0, 1.5])#flame3 0.9
# humanoid = p.loadURDF("urdf/simbicon_urdf/demo.urdf")
# gravId = p.addUserDebugParameter("gravity",-10,10,-10)
gravId = p.addUserDebugParameter("gravity",-10,10,0)
jointIds=[]
paramIds=[]

# add box 
# test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2,1,0.1],rgbaColor=[1, 0, 0, 1])
# test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2,1,0.1])
# test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
# baseVisualShapeIndex=test_visual, basePosition = [-0.15, 0, 0])


p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid,-1,linearDamping=0, angularDamping=0)

# jointAngles=[0,0,1.0204,-1.97,-0.084,2.06,-1.9,0,0,1.0204,-1.97,-0.084,2.06,-1.9,0]
jointAngles=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# # rev_jointAngles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
activeJoint=0
for j in range (p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid,j,linearDamping=0, angularDamping=0)
    info = p.getJointInfo(humanoid,j)
    jointName = info[1]
    jointType = info[2]
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        activeJoint+=1
        jointIds.append(j)
        paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,jointAngles[activeJoint]))
        # p.resetJointState(humanoid, j, jointAngles[activeJoint])

p.setRealTimeSimulation(1)
while(1):
    p.getCameraImage(320,200)
    p.setGravity(0,0,p.readUserDebugParameter(gravId))

    # joint_states = p.getJointStates(humanoid,range(1))
    info = p.getJointInfo(humanoid,0)
    print(info)
    
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(humanoid,jointIds[i],p.POSITION_CONTROL,targetPos, force=140.)
        p.setTimeStep(0.1)

    time.sleep(0.01)
    #   time.sleep(10)