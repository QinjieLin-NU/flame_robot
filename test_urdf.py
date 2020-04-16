import pybullet as p
p.connect(p.GUI)
# p.loadURDF("urdf/legged_robot/legged_robot.urdf")
p.loadURDF("urdf/plane.urdf",basePosition=[0,0,-1.5])
# p.loadURDF("urdf/simbicon_urdf/biped2d.urdf")
p.loadURDF("urdf/simbicon_urdf/humanoid_nohead.urdf")
# gravId = p.addUserDebugParameter("gravity",-10,10,-10)
p.setPhysicsEngineParameter(numSolverIterations=100)

while (1):
  p.stepSimulation()
